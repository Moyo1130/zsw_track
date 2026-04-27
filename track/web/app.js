/* eslint-disable no-console */

/**
 * 期望 WS payload（与 Python FrameInput.from_yolo_ws_payload 一致）：
 * {
 *   code: 0,
 *   msg: "ok",
 *   data: {
 *     timestamp: number,
 *     image_width: number,
 *     image_height: number,
 *     persons: [{ confidence, track_id, bbox: { x1,y1,x2,y2 } }]
 *   }
 * }
 */

function clamp(v, lo, hi) {
  return v < lo ? lo : v > hi ? hi : v;
}

function fmt(n, d = 3) {
  if (n == null || Number.isNaN(n)) return "-";
  return Number(n).toFixed(d);
}

function parseNumber(x, fallback) {
  const n = Number(x);
  return Number.isFinite(n) ? n : fallback;
}

function buildWsUrl(rawUrl, token) {
  const url = rawUrl.trim();
  if (!url) return "";
  if (!token) return url;
  if (url.includes("token=")) return url;
  const sep = url.includes("?") ? "&" : "?";
  return `${url}${sep}token=${encodeURIComponent(token)}`;
}

function selectTarget(persons, mode) {
  if (!Array.isArray(persons) || persons.length === 0) return null;
  if (mode === "largest") {
    let best = persons[0];
    let bestArea = -1;
    for (const p of persons) {
      const b = p?.bbox;
      if (!b) continue;
      const area = Math.max(0, (b.x2 - b.x1) * (b.y2 - b.y1));
      if (area > bestArea) {
        bestArea = area;
        best = p;
      }
    }
    return best;
  }
  if (mode === "highest_conf") {
    let best = persons[0];
    let bestC = -1;
    for (const p of persons) {
      const c = Number(p?.confidence ?? 0);
      if (c > bestC) {
        bestC = c;
        best = p;
      }
    }
    return best;
  }
  return persons[0];
}

class CenterFollowController {
  constructor() {
    this.prevLin = 0;
    this.prevAng = 0;
  }

  compute(frameW, frameH, bbox, params) {
    const iw = Math.max(1, frameW);
    const ih = Math.max(1, frameH);

    const x1 = bbox.x1;
    const y1 = bbox.y1;
    const x2 = bbox.x2;
    const y2 = bbox.y2;

    const cx = (x1 + x2) / 2;
    const bboxH = Math.max(0, y2 - y1);
    const offsetPx = cx - iw / 2;
    let normOffset = offsetPx / (iw / 2);
    normOffset = clamp(normOffset, -1, 1);

    let fillRatio = bboxH / ih;
    fillRatio = clamp(fillRatio, 0, 1);

    const kAng = params.kAng;
    const angMax = params.angMax;
    const desiredFill = params.desiredFill;
    const kLin = params.kLin;
    const linMax = params.linMax;
    const alpha = clamp(params.alpha, 0, 0.98);

    const angNow = clamp(-kAng * normOffset, -angMax, angMax);

    const distanceError = desiredFill - fillRatio; // >0 means "too far" -> move forward
    const linRaw = kLin * distanceError;
    let linNow = clamp(linRaw, 0, linMax);
    linNow *= 1 - Math.abs(normOffset); // off-center -> slow down

    const lin = alpha * this.prevLin + (1 - alpha) * linNow;
    const ang = alpha * this.prevAng + (1 - alpha) * angNow;
    this.prevLin = lin;
    this.prevAng = ang;

    return {
      linear_x: lin,
      angular_z: ang,
      offsetPx,
      normOffset,
      fillRatio,
      reason: `center-follow: offset=${offsetPx.toFixed(1)}px norm=${normOffset.toFixed(3)} fill=${fillRatio.toFixed(
        2
      )} -> lin=${lin.toFixed(3)} ang=${ang.toFixed(3)}`,
    };
  }
}

const el = {
  wsUrl: document.getElementById("wsUrl"),
  token: document.getElementById("token"),
  btnConnect: document.getElementById("btnConnect"),
  btnDisconnect: document.getElementById("btnDisconnect"),
  connDot: document.getElementById("connDot"),
  connText: document.getElementById("connText"),

  kAng: document.getElementById("kAng"),
  angMax: document.getElementById("angMax"),
  desiredFill: document.getElementById("desiredFill"),
  kLin: document.getElementById("kLin"),
  linMax: document.getElementById("linMax"),
  alpha: document.getElementById("alpha"),
  targetMode: document.getElementById("targetMode"),

  canvas: document.getElementById("canvas"),
  overlayText: document.getElementById("overlayText"),

  ts: document.getElementById("ts"),
  frameWH: document.getElementById("frameWH"),
  personsCount: document.getElementById("personsCount"),
  trackId: document.getElementById("trackId"),

  bbox: document.getElementById("bbox"),
  offsetPx: document.getElementById("offsetPx"),
  offsetNorm: document.getElementById("offsetNorm"),
  fillRatio: document.getElementById("fillRatio"),

  cmdLin: document.getElementById("cmdLin"),
  cmdAng: document.getElementById("cmdAng"),
  reason: document.getElementById("reason"),
};

const ctx = el.canvas.getContext("2d");
const controller = new CenterFollowController();
let ws = null;
let lastFrame = null;
let reconnectTimer = null;
let reconnectAttempts = 0;
let manualDisconnect = false;

function setConnState(state, text) {
  el.connDot.classList.remove("good", "bad");
  if (state === "good") el.connDot.classList.add("good");
  if (state === "bad") el.connDot.classList.add("bad");
  el.connText.textContent = text;
}

function clearReconnect() {
  if (reconnectTimer) {
    clearTimeout(reconnectTimer);
    reconnectTimer = null;
  }
}

function scheduleReconnect() {
  if (manualDisconnect) return;
  clearReconnect();
  reconnectAttempts += 1;
  const backoffMs = clamp(400 * 2 ** Math.min(reconnectAttempts, 6), 400, 15000);
  setConnState("bad", `已断开，${Math.round(backoffMs / 1000)}s 后重连…`);
  reconnectTimer = setTimeout(() => {
    connect({ isAutoReconnect: true });
  }, backoffMs);
}

function getParams() {
  return {
    kAng: parseNumber(el.kAng.value, 1.1),
    angMax: parseNumber(el.angMax.value, 0.55),
    desiredFill: parseNumber(el.desiredFill.value, 0.7),
    kLin: parseNumber(el.kLin.value, 0.9),
    linMax: parseNumber(el.linMax.value, 0.25),
    alpha: parseNumber(el.alpha.value, 0.75),
  };
}

function resizeCanvasToFrame(w, h) {
  const cw = Math.max(1, Math.round(w));
  const ch = Math.max(1, Math.round(h));
  if (el.canvas.width !== cw || el.canvas.height !== ch) {
    el.canvas.width = cw;
    el.canvas.height = ch;
  }
}

function draw(frameW, frameH, persons, selected, computed) {
  resizeCanvasToFrame(frameW, frameH);

  // background
  ctx.clearRect(0, 0, el.canvas.width, el.canvas.height);
  ctx.fillStyle = "rgba(8, 10, 20, 1)";
  ctx.fillRect(0, 0, el.canvas.width, el.canvas.height);

  // center line
  ctx.strokeStyle = "rgba(122, 162, 255, 0.6)";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(frameW / 2, 0);
  ctx.lineTo(frameW / 2, frameH);
  ctx.stroke();

  // draw all persons (dim)
  if (Array.isArray(persons)) {
    for (const p of persons) {
      const b = p?.bbox;
      if (!b) continue;
      ctx.strokeStyle = "rgba(255,255,255,0.28)";
      ctx.lineWidth = 2;
      ctx.strokeRect(b.x1, b.y1, b.x2 - b.x1, b.y2 - b.y1);
    }
  }

  // selected target
  if (selected?.bbox) {
    const b = selected.bbox;
    ctx.strokeStyle = "rgba(61, 220, 151, 0.95)";
    ctx.lineWidth = 3;
    ctx.strokeRect(b.x1, b.y1, b.x2 - b.x1, b.y2 - b.y1);

    const cx = (b.x1 + b.x2) / 2;
    const cy = (b.y1 + b.y2) / 2;
    ctx.fillStyle = "rgba(61, 220, 151, 0.95)";
    ctx.beginPath();
    ctx.arc(cx, cy, 4, 0, Math.PI * 2);
    ctx.fill();

    // arrow for angular direction
    const ang = computed?.angular_z ?? 0;
    const arrowLen = 40;
    const dx = Math.sign(ang) * 18;
    ctx.strokeStyle = "rgba(255, 93, 93, 0.9)";
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(frameW / 2, frameH - 24);
    ctx.lineTo(frameW / 2 + dx, frameH - 24 - arrowLen);
    ctx.stroke();
  }

  // overlay text
  const lines = [];
  if (computed) {
    lines.push(`linear_x: ${computed.linear_x.toFixed(3)}`);
    lines.push(`angular_z: ${computed.angular_z.toFixed(3)}`);
    lines.push(`norm_offset: ${computed.normOffset.toFixed(3)}`);
    lines.push(`fill_ratio: ${computed.fillRatio.toFixed(3)}`);
  } else {
    lines.push("等待数据…");
  }
  el.overlayText.textContent = lines.join("\n");
}

function updateTelemetry(payload, selected, computed) {
  el.ts.textContent = payload?.data?.timestamp ?? "-";
  el.frameWH.textContent = `${payload?.data?.image_width ?? "-"}×${payload?.data?.image_height ?? "-"}`;
  el.personsCount.textContent = Array.isArray(payload?.data?.persons) ? payload.data.persons.length : "-";
  el.trackId.textContent = selected?.track_id ?? "-";

  if (selected?.bbox) {
    const b = selected.bbox;
    el.bbox.textContent = `x1=${b.x1.toFixed(1)} y1=${b.y1.toFixed(1)} x2=${b.x2.toFixed(1)} y2=${b.y2.toFixed(1)}`;
  } else {
    el.bbox.textContent = "-";
  }

  el.offsetPx.textContent = computed ? `${computed.offsetPx.toFixed(1)} px` : "-";
  el.offsetNorm.textContent = computed ? computed.normOffset.toFixed(3) : "-";
  el.fillRatio.textContent = computed ? computed.fillRatio.toFixed(3) : "-";

  el.cmdLin.textContent = computed ? fmt(computed.linear_x, 3) : "-";
  el.cmdAng.textContent = computed ? fmt(computed.angular_z, 3) : "-";
  el.reason.textContent = computed ? computed.reason : "-";
}

function safeParseJson(str) {
  try {
    return JSON.parse(str);
  } catch {
    return null;
  }
}

function handleMessage(raw) {
  const payload = typeof raw === "string" ? safeParseJson(raw) : null;
  if (!payload) return;
  if (payload?.type === "ping") {
    // 机器狗 WS 可能要求 pong 保活（Python 版本 track_live.py 会回 pong）
    try {
      ws?.send(JSON.stringify({ type: "pong", ts: payload?.ts }));
    } catch {
      // ignore
    }
    return;
  }
  if (payload?.code !== 0) return;

  const data = payload.data;
  if (!data || typeof data !== "object") return;

  const persons = Array.isArray(data.persons) ? data.persons : [];
  const mode = el.targetMode.value;
  const selected = selectTarget(persons, mode);

  const frameW = Number(data.image_width ?? 1280);
  const frameH = Number(data.image_height ?? 720);

  let computed = null;
  if (selected?.bbox && typeof selected.bbox === "object") {
    computed = controller.compute(frameW, frameH, selected.bbox, getParams());
  } else {
    // no target: decay commands to 0 (smoothly)
    const alpha = clamp(parseNumber(el.alpha.value, 0.75), 0, 0.98);
    controller.prevLin = alpha * controller.prevLin;
    controller.prevAng = alpha * controller.prevAng;
  }

  lastFrame = { payload, persons, selected, computed, frameW, frameH };
  draw(frameW, frameH, persons, selected, computed);
  updateTelemetry(payload, selected, computed);
}

function disconnect() {
  manualDisconnect = true;
  clearReconnect();
  if (ws) {
    try {
      ws.close();
    } catch {
      // ignore
    }
    ws = null;
  }
  el.btnConnect.disabled = false;
  el.btnDisconnect.disabled = true;
  setConnState("bad", "未连接");
}

function connect({ isAutoReconnect } = { isAutoReconnect: false }) {
  manualDisconnect = false;
  const url = buildWsUrl(el.wsUrl.value, el.token.value.trim());
  if (!url) return;

  disconnect();
  manualDisconnect = false;

  if (!isAutoReconnect) reconnectAttempts = 0;
  setConnState("", "连接中…");
  el.btnConnect.disabled = true;
  el.btnDisconnect.disabled = false;

  ws = new WebSocket(url);
  ws.onopen = () => {
    setConnState("good", "已连接");
    clearReconnect();
    reconnectAttempts = 0;
  };
  ws.onerror = () => {
    setConnState("bad", "连接错误");
  };
  ws.onclose = () => {
    // 不调用 disconnect()（会把 manualDisconnect 置 true），这里做自动重连
    el.btnConnect.disabled = false;
    el.btnDisconnect.disabled = true;
    setConnState("bad", "已断开");
    scheduleReconnect();
  };
  ws.onmessage = (ev) => {
    handleMessage(ev.data);
  };
}

el.btnConnect.addEventListener("click", connect);
el.btnDisconnect.addEventListener("click", disconnect);

// param changes: redraw overlay immediately
for (const id of ["kAng", "angMax", "desiredFill", "kLin", "linMax", "alpha", "targetMode"]) {
  const node = document.getElementById(id);
  node.addEventListener("input", () => {
    if (!lastFrame) return;
    if (!lastFrame.selected?.bbox) return;
    lastFrame.computed = controller.compute(lastFrame.frameW, lastFrame.frameH, lastFrame.selected.bbox, getParams());
    draw(lastFrame.frameW, lastFrame.frameH, lastFrame.persons, lastFrame.selected, lastFrame.computed);
    updateTelemetry(lastFrame.payload, lastFrame.selected, lastFrame.computed);
  });
}

// first paint
draw(1280, 720, [], null, null);
setConnState("bad", "未连接");

