from __future__ import annotations

from dataclasses import dataclass
from typing import Any


TRACKING = "TRACKING"
TRACKING_BUFFER = "TRACKING_BUFFER"
TRACKING_LOST = "TRACKING_LOST"
IDLE = "IDLE"


@dataclass(frozen=True)
class BBox:
    x1: float
    y1: float
    x2: float
    y2: float

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> "BBox":
        return cls(
            x1=float(payload["x1"]),
            y1=float(payload["y1"]),
            x2=float(payload["x2"]),
            y2=float(payload["y2"]),
        )


@dataclass(frozen=True)
class Detection:
    class_name: str
    score: float
    bbox: BBox
    track_id: int | None = None

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> "Detection":
        raw_tid = payload.get("track_id")
        return cls(
            class_name=str(payload["class_name"]),
            score=float(payload["score"]),
            bbox=BBox.from_dict(payload["bbox"]),
            track_id=None if raw_tid is None else int(raw_tid),
        )


@dataclass(frozen=True)
class FrameInput:
    timestamp: int
    image_width: int
    image_height: int
    detections: list[Detection]

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> "FrameInput":
        return cls(
            timestamp=int(payload["timestamp"]),
            image_width=int(payload["image_width"]),
            image_height=int(payload["image_height"]),
            detections=[Detection.from_dict(item) for item in payload["detections"]],
        )

    @classmethod
    def from_http_response(cls, payload: dict[str, Any]) -> "FrameInput":
        """Parse YOLO HTTP response format with code/message wrapper.

        Args:
            payload: HTTP response JSON with fields: code, message, timestamp,
                     image_width, image_height, detections

        Raises:
            ValueError: If code != 0 or missing required fields
        """
        if payload.get("code") != 0:
            raise ValueError(
                f"YOLO service error: code={payload.get('code')}, "
                f"message={payload.get('message', 'unknown')}"
            )
        return cls.from_dict(payload)

    @classmethod
    def from_yolo_ws_payload(cls, payload: dict[str, Any]) -> "FrameInput":
        """机器狗 YOLO WebSocket 推送格式（见 YOLO客户端开发文档.md）：code/msg/data.persons[]."""
        if payload.get("code") != 0:
            raise ValueError(
                f"YOLO WS error: code={payload.get('code')} msg={payload.get('msg')}"
            )
        data = payload.get("data")
        if not isinstance(data, dict):
            raise ValueError("YOLO WS: missing data")

        raw_ts = data.get("timestamp", 0)
        # 文档定义：timestamp 为 Unix 时间戳（秒，float）。
        # 兼容：若上游误传毫秒（>=1e12），则直接按毫秒使用；否则按秒*1000 转为毫秒 int。
        try:
            tf = float(raw_ts)
        except (TypeError, ValueError) as e:
            raise ValueError(f"YOLO WS: invalid timestamp={raw_ts!r}") from e
        timestamp = int(tf) if tf >= 1e12 else int(tf * 1000.0)

        iw = int(data["image_width"])
        ih = int(data["image_height"])
        detections: list[Detection] = []
        # 以 persons 为准：部分实现只填 persons、不置 detected，或二者不一致
        persons_raw = data.get("persons")
        if not isinstance(persons_raw, list):
            persons_raw = []
        for p in persons_raw:
            if not isinstance(p, dict):
                continue
            bbox = p.get("bbox")
            if not isinstance(bbox, dict):
                continue
            det_dict: dict[str, Any] = {
                "class_name": "person",
                "score": float(p.get("confidence", 0.0)),
                "bbox": bbox,
                "track_id": p.get("track_id"),
            }
            detections.append(Detection.from_dict(det_dict))

        return cls(
            timestamp=timestamp,
            image_width=iw,
            image_height=ih,
            detections=detections,
        )


@dataclass(frozen=True)
class TargetMetrics:
    bbox_center_x: float
    bbox_center_y: float
    bbox_width: float
    bbox_height: float
    bbox_area: float
    center_offset_x: float


@dataclass(frozen=True)
class TwistCommand:
    linear_x: float
    angular_z: float
    reason: str
    state: str
    previous_state: str
    lost_frame_count: int
    selected_detection: Detection | None = None
    target_metrics: TargetMetrics | None = None

    def to_rostopic_command(self, rate: int = 10) -> str:
        return (
            'rostopic pub /cmd_vel geometry_msgs/Twist "linear: '
            f"{{x: {self._format_value(self.linear_x)}, y: 0, z: 0}}\n"
            "angular: "
            f"{{x: 0, y: 0, z: {self._format_value(self.angular_z)}}}\" "
            f"-r {rate}"
        )

    @staticmethod
    def _format_value(value: float) -> str:
        if abs(value) < 1e-9:
            return "0.0"
        text = f"{value:.2f}".rstrip("0").rstrip(".")
        if "." not in text:
            text = f"{text}.0"
        return text


class P4TrackerController:
    def __init__(
        self,
        *,
        center_x: float = 640.0,
        # 采用连续比例控制：目标始终拉向画面中心，避免“分段阈值跳档”带来的抖动。
        # 角速度：PD 控制（减少来回摇摆/冲过头）
        # angular_z = clamp(-(kp_ang * e + kd_ang * de/dt), [-ang_max, +ang_max])
        kp_ang: float = 0.6,
        kd_ang: float = 0.10,
        ang_max: float = 0.28,
        # 前进速度：围绕期望“跟随距离”比例做连续控制，并在偏离中心时自动降速。
        # v = clamp(k_lin * (desired_fill - fill_ratio), 0, lin_max) * (1 - |normalized_offset|)
        desired_fill_ratio: float = 0.70,
        k_lin: float = 0.9,
        lin_max: float = 0.18,
        # 指令平滑：一阶低通滤波，减少 YOLO 抖动造成的“蛇形”。
        # cmd = alpha*prev + (1-alpha)*now
        smooth_alpha: float = 0.82,
        # 丢失处理：短暂丢失时延用上一帧指令（衰减），长时间丢失进入搜索（慢速转圈/摆动）
        lost_hold_frames: int = 2,
        lost_hold_decay: float = 0.85,
        search_ang: float = 0.25,
        search_period_s: float = 1.2,
        lost_timeout: int = 10,
    ) -> None:
        self.center_x = center_x
        self.kp_ang = float(kp_ang)
        self.kd_ang = float(kd_ang)
        self.ang_max = float(ang_max)
        self.desired_fill_ratio = float(desired_fill_ratio)
        self.k_lin = float(k_lin)
        self.lin_max = float(lin_max)
        self.smooth_alpha = float(smooth_alpha)
        self.lost_hold_frames = int(lost_hold_frames)
        self.lost_hold_decay = float(lost_hold_decay)
        self.search_ang = float(search_ang)
        self.search_period_s = float(search_period_s)
        self.lost_timeout = lost_timeout

        self.locked_track_id: int | None = None
        self.lost_frame_count = 0
        self.tracking_state = IDLE
        self._prev_linear_x = 0.0
        self._prev_angular_z = 0.0
        self._prev_error_x = 0.0
        self._prev_ts_ms: int | None = None

    @staticmethod
    def _clamp(value: float, lo: float, hi: float) -> float:
        return lo if value < lo else hi if value > hi else value

    def _smooth(self, now: float, prev: float) -> float:
        alpha = self._clamp(self.smooth_alpha, 0.0, 0.98)
        return alpha * prev + (1.0 - alpha) * now

    def compute_command(self, frame: FrameInput) -> TwistCommand:
        previous_state = self.tracking_state
        target = self._select_target(frame)
        if target is None:
            return self._handle_missing_target(previous_state)

        # 目标重新出现/重新锁定时，清理上一段的平滑历史（尤其是搜索阶段的角速度），
        # 否则 smooth_alpha 很大时会出现“目标在右却仍朝左转一会儿”的残留。
        if previous_state != TRACKING:
            self._prev_linear_x = 0.0
            self._prev_angular_z = 0.0
            self._prev_error_x = 0.0
            self._prev_ts_ms = None

        self.lost_frame_count = 0
        if self.locked_track_id is None and target.track_id is not None:
            self.locked_track_id = target.track_id

        metrics = self._build_metrics(target, frame.image_width)
        self.tracking_state = TRACKING
        iw = max(frame.image_width, 1)
        ih = max(frame.image_height, 1)

        # 目标中心偏移（像素）→ 归一化偏移 [-1, 1]
        norm_offset = metrics.center_offset_x / (iw / 2.0)
        norm_offset = self._clamp(norm_offset, -1.0, 1.0)

        # 目标占屏比例：用 bbox 高度 / 画面高度作距离代理（连续）
        fill_ratio = metrics.bbox_height / ih
        fill_ratio = 0.0 if fill_ratio < 0.0 else 1.0 if fill_ratio > 1.0 else fill_ratio

        # PD 角速度控制：P 拉回中心，D 抑制冲过头带来的来回摆动
        ts_ms = int(frame.timestamp)
        dt = 0.1
        if self._prev_ts_ms is not None:
            raw_dt = (ts_ms - self._prev_ts_ms) / 1000.0
            # 避免 dt 太小或异常跳变导致 D 项爆炸
            dt = self._clamp(raw_dt, 0.02, 0.5)
        de = (norm_offset - self._prev_error_x) / dt
        self._prev_error_x = norm_offset
        self._prev_ts_ms = ts_ms

        ang_pd = -(self.kp_ang * norm_offset + self.kd_ang * de)
        ang_now = self._clamp(ang_pd, -self.ang_max, self.ang_max)

        # 连续前进：越远（fill 越小）越前进，越近越减速；偏离中心时也自动减速，先对齐再追
        distance_error = self.desired_fill_ratio - fill_ratio
        lin_raw = self.k_lin * distance_error
        lin_now = self._clamp(lin_raw, 0.0, self.lin_max)
        lin_now *= 1.0 - abs(norm_offset)

        # 平滑输出，减少检测抖动导致的指令抖动
        lin = self._smooth(lin_now, self._prev_linear_x)
        ang = self._smooth(ang_now, self._prev_angular_z)
        self._prev_linear_x = lin
        self._prev_angular_z = ang

        return TwistCommand(
            linear_x=lin,
            angular_z=ang,
            reason=(
                "center-follow: "
                f"offset_px={metrics.center_offset_x:.1f} norm={norm_offset:+.3f} "
                f"fill={fill_ratio:.2f} desired={self.desired_fill_ratio:.2f} "
                f"-> lin={lin:.3f} ang={ang:.3f}"
            ),
            state=self.tracking_state,
            previous_state=previous_state,
            lost_frame_count=self.lost_frame_count,
            selected_detection=target,
            target_metrics=metrics,
        )

    def _select_target(self, frame: FrameInput) -> Detection | None:
        people = [item for item in frame.detections if item.class_name == "person"]
        if not people:
            return None

        if self.locked_track_id is not None:
            for detection in people:
                if (
                    detection.track_id is not None
                    and detection.track_id == self.locked_track_id
                ):
                    return detection
            return None

        # 未锁定 track_id 时，优先选择置信度（score）最高的目标，避免 persons 顺序波动导致跟随目标跳变。
        return max(people, key=lambda det: det.score)

    def _handle_missing_target(self, previous_state: str) -> TwistCommand:
        if self.tracking_state == TRACKING_LOST:
            return TwistCommand(
                linear_x=0.0,
                angular_z=0.0,
                reason="target missing (tracking lost; hold)",
                state=self.tracking_state,
                previous_state=previous_state,
                lost_frame_count=self.lost_timeout,
            )

        self.lost_frame_count += 1
        if self.lost_frame_count >= self.lost_timeout:
            self.tracking_state = TRACKING_LOST
            self.locked_track_id = None
            reason = (
                f"target missing for {self.lost_frame_count} consecutive frames; "
                "tracking lost and controller reset"
            )
        else:
            self.tracking_state = TRACKING_BUFFER
            if self.lost_frame_count <= max(0, self.lost_hold_frames):
                # 短暂丢失：延用上一帧指令并衰减，避免“瞬间停车→更难找回”
                decay = self._clamp(self.lost_hold_decay, 0.0, 0.98)
                self._prev_linear_x *= decay
                self._prev_angular_z *= decay
                return TwistCommand(
                    linear_x=self._prev_linear_x,
                    angular_z=self._prev_angular_z,
                    reason=(
                        f"target missing ({self.lost_frame_count}/{self.lost_timeout}); "
                        f"hold last cmd with decay={decay:.2f}"
                    ),
                    state=self.tracking_state,
                    previous_state=previous_state,
                    lost_frame_count=self.lost_frame_count,
                )

            # 更久丢失：慢速扫描搜寻（左右摆动），防止站着不动越丢越丢
            period = max(0.4, float(self.search_period_s))
            # 用 lost_frame_count 近似时间（假设 ~10Hz）；够用且无需依赖真实时钟
            t = (self.lost_frame_count - self.lost_hold_frames) * 0.1
            phase = int(t / period) % 2
            direction = 1.0 if phase == 0 else -1.0
            ang = direction * self._clamp(abs(self.search_ang), 0.05, self.ang_max)
            self._prev_linear_x = 0.0
            self._prev_angular_z = ang
            return TwistCommand(
                linear_x=0.0,
                angular_z=ang,
                reason=(
                    f"target missing ({self.lost_frame_count}/{self.lost_timeout}); "
                    "search by slow yaw scan"
                ),
                state=self.tracking_state,
                previous_state=previous_state,
                lost_frame_count=self.lost_frame_count,
            )

        return TwistCommand(
            linear_x=0.0,
            angular_z=0.0,
            reason=reason,
            state=self.tracking_state,
            previous_state=previous_state,
            lost_frame_count=self.lost_frame_count,
        )

    def _build_metrics(self, detection: Detection, image_width: int) -> TargetMetrics:
        bbox_width = detection.bbox.x2 - detection.bbox.x1
        bbox_height = detection.bbox.y2 - detection.bbox.y1
        bbox_center_x = (detection.bbox.x1 + detection.bbox.x2) / 2.0
        bbox_center_y = (detection.bbox.y1 + detection.bbox.y2) / 2.0
        cx = image_width / 2.0
        return TargetMetrics(
            bbox_center_x=bbox_center_x,
            bbox_center_y=bbox_center_y,
            bbox_width=bbox_width,
            bbox_height=bbox_height,
            bbox_area=bbox_width * bbox_height,
            center_offset_x=bbox_center_x - cx,
        )
