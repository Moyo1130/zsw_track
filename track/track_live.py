#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Tracker 联机控狗（简化版）

FrameInput → P4TrackerController → TwistCommand → 轴速度映射 → totalController.Controller

分步验证（建议顺序）：
  1) 映射是否正确（不连狗）：
       python track_live.py --check-mapping
  2) 底层 UDP 能否驱狗（与 Tracker 无关，真机、空地）：
       python udp_motion_demo.py --ip <IP> --port <PORT>
       或：python track_live.py --smoke-udp --ip <IP> --port <PORT>
  3) 完整闭环：python track_live.py --dry-run / 去 --dry-run 接 YOLO 或 scenarios

用法示例：
  # JSON scenarios → Tracker → UDP（先 dry-run 再打狗）
  python track_live.py --dry-run --scenarios samples/scenarios.json
  python track_live.py --scenarios samples/scenarios.json --delay 0.15 --ip 10.69.235.139
  # 单帧 JSON 打一次动作（适合一条一条试）：
  python track_json_one.py samples/frames/turn_left.json --dry-run
  python track_live.py --yolo-url http://127.0.0.1:8080/detections --poll-hz 10
  # 真机 YOLO WebSocket（token 在 .env API_TOKEN，见 YOLO客户端开发文档.md）
  python track_live.py --dry-run --yolo-ws ws://10.61.248.65:8001/ws/detection
  python track_live.py --yolo-ws ws://10.61.248.65:8001/ws/detection
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import sys
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

from p4_tracker.controller import FrameInput, P4TrackerController, TwistCommand
from p4_tracker.controller import TRACKING
from totalController import Controller
from udp_motion_demo import SoftExitController, run_axis_motion, safe_stop

DEFAULT_IP = "10.61.248.65"
DEFAULT_YOLO_WS = f"ws://{DEFAULT_IP}:8001/ws/detection"
DEFAULT_PORT = 43893
ZERO_EPS = 1e-6
# 与 track_json_one 一致：enable 后给固件一点时间再发轴流；仅在「静止→运动」首帧需要
_ENABLE_SETTLE_S = 0.15

# totalController.move 注释：前后轴约 ±0.2、转向约 ±0.29 以内为死区，再小则不动
# （6553/32767≈0.20，9553/32767≈0.29）抬升值必须 **严格大于** 死区边界，否则固件直接忽略该轴。
# 前进只做正向跟随，不做后退；左右继续沿用现有稳定的转向门控。
_MIN_EFFECTIVE_FORWARD = 0.22
# turn_axis_min 默认用 ~0.32 越过 0.29，确保真机会转起来。
_MIN_EFFECTIVE_TURN = 0.32


def twist_to_forward_axis(
    cmd: TwistCommand,
    *,
    forward_gain: float = 2.0,
    forward_axis_min: float = _MIN_EFFECTIVE_FORWARD,
    forward_axis_max: float = 0.32,
) -> float:
    """只允许前进：linear_x>0 时映射到前进轴，并跨过固件前进死区（约 0.20）。"""
    forward = max(0.0, min(1.0, cmd.linear_x * forward_gain))
    fmax = max(0.0, min(1.0, float(forward_axis_max)))
    fmin = max(0.0, min(fmax, float(forward_axis_min)))
    forward = max(0.0, min(fmax, forward))
    if cmd.linear_x > ZERO_EPS and 0.0 < forward < fmin:
        forward = fmin
    return forward


def twist_to_turn_axis(
    cmd: TwistCommand,
    *,
    turn_gain: float = 1.0,
    turn_axis_min: float = 0.32,
    turn_axis_max: float = 0.40,
) -> float:
    """
    yaw-only：TwistCommand → turn_speed（-1~1）。

    - 狗：turn_speed > 0 为右转
    - Tracker/ROS：angular_z > 0 常为左转 → turn_speed = -angular_z
    - turn_axis_min/turn_axis_max 用于把转向限制在“小幅度”，同时跨越固件转向死区（约 0.29）
    """
    turn = max(-1.0, min(1.0, (-cmd.angular_z) * turn_gain))
    tmax = max(0.0, min(1.0, float(turn_axis_max)))
    tmin = max(0.0, min(tmax, float(turn_axis_min)))
    turn = max(-tmax, min(tmax, turn))
    if abs(cmd.angular_z) > ZERO_EPS and 0.0 < abs(turn) < tmin:
        turn = math.copysign(tmin, turn)
    return turn


def twist_to_move(
    cmd: TwistCommand,
    *,
    forward_gain: float = 2.0,
    forward_axis_min: float = _MIN_EFFECTIVE_FORWARD,
    forward_axis_max: float = 0.32,
    turn_gain: float = 1.0,
    turn_axis_min: float = _MIN_EFFECTIVE_TURN,
    turn_axis_max: float = 0.40,
) -> tuple[float, float, float]:
    """TwistCommand -> (forward, side, turn)。当前只用前进和转向。"""
    return (
        twist_to_forward_axis(
            cmd,
            forward_gain=forward_gain,
            forward_axis_min=forward_axis_min,
            forward_axis_max=forward_axis_max,
        ),
        0.0,
        twist_to_turn_axis(
            cmd,
            turn_gain=turn_gain,
            turn_axis_min=turn_axis_min,
            turn_axis_max=turn_axis_max,
        ),
    )


def is_stationary(cmd: TwistCommand) -> bool:
    return abs(cmd.linear_x) < ZERO_EPS and abs(cmd.angular_z) < ZERO_EPS


def has_live_target(cmd: TwistCommand) -> bool:
    return (cmd.state == TRACKING) and (cmd.selected_detection is not None)


def compute_dynamic_start_norm(
    *,
    fill: float,
    start_norm: float,
    desired_fill: float,
    far_start_boost: float,
) -> float:
    """目标越远（fill 越小），转向触发阈值越高，减少远距离小偏差带来的过度修正。"""
    base = max(0.0, float(start_norm))
    desired = max(0.05, float(desired_fill))
    boost = max(0.0, float(far_start_boost))
    if boost < ZERO_EPS or fill >= desired:
        return base

    far_ratio = (desired - max(0.0, fill)) / desired
    dynamic = base + boost * far_ratio
    return max(base, min(0.95, dynamic))


@dataclass
class TurnGateState:
    action_until: float = 0.0
    cooldown_until: float = 0.0
    sent_zero_after_action: bool = True
    engaged: bool = False
    seen_streak: int = 0
    active_turn: float = 0.0


def resolve_motion_axes(
    cmd: TwistCommand,
    frame: FrameInput,
    gate: TurnGateState,
    *,
    now: float,
    forward_gain: float,
    forward_axis_min: float,
    forward_axis_max: float,
    turn_gain: float,
    turn_axis_min: float,
    turn_axis_max: float,
    start_norm: float,
    stop_norm: float,
    desired_fill: float,
    far_start_boost: float,
    seen_n: int,
    action_s: float,
    cooldown_s: float,
) -> tuple[float, float, float]:
    has_target = has_live_target(cmd)
    forward = 0.0
    if has_target:
        forward = twist_to_forward_axis(
            cmd,
            forward_gain=forward_gain,
            forward_axis_min=forward_axis_min,
            forward_axis_max=forward_axis_max,
        )

    if now < gate.action_until:
        return (forward, 0.0, gate.active_turn)

    if (not gate.sent_zero_after_action) and now >= gate.action_until:
        gate.sent_zero_after_action = True
        gate.cooldown_until = now + max(0.0, cooldown_s)
        gate.active_turn = 0.0
        return (forward, 0.0, 0.0)

    if now < gate.cooldown_until:
        return (forward, 0.0, 0.0)

    if has_target and cmd.target_metrics is not None:
        iw = max(frame.image_width, 1)
        ih = max(frame.image_height, 1)
        norm = cmd.target_metrics.center_offset_x / (iw / 2.0)
        fill = cmd.target_metrics.bbox_height / ih
        effective_start_norm = compute_dynamic_start_norm(
            fill=fill,
            start_norm=start_norm,
            desired_fill=desired_fill,
            far_start_boost=far_start_boost,
        )
        if abs(norm) <= stop_norm:
            gate.engaged = False
            gate.seen_streak = 0
        elif abs(norm) >= effective_start_norm:
            gate.engaged = True
            gate.seen_streak += 1
    else:
        gate.engaged = False
        gate.seen_streak = 0

    turn = 0.0
    if gate.engaged and gate.seen_streak >= seen_n and abs(cmd.angular_z) > ZERO_EPS:
        turn = twist_to_turn_axis(
            cmd,
            turn_gain=turn_gain,
            turn_axis_min=turn_axis_min,
            turn_axis_max=turn_axis_max,
        )
        gate.active_turn = turn
        gate.sent_zero_after_action = False
        gate.action_until = now + max(0.0, action_s)
        gate.seen_streak = 0

    return (forward, 0.0, turn)


def apply_motion_axes(
    robot: Controller | None,
    *,
    forward_speed: float,
    side_speed: float,
    turn_speed: float,
    dry_run: bool,
    cmd: TwistCommand | None = None,
) -> None:
    if dry_run:
        if cmd is not None:
            print(
                "  [dry-run] "
                f"twist linear_x={cmd.linear_x:.3f} angular_z={cmd.angular_z:.3f} "
                f"-> forward={forward_speed:.3f} turn={turn_speed:.3f}"
            )
        else:
            print(
                f"  [dry-run] forward={forward_speed:.3f} side={side_speed:.3f} turn={turn_speed:.3f}"
            )
        return

    assert robot is not None
    if abs(forward_speed) < ZERO_EPS and abs(side_speed) < ZERO_EPS and abs(turn_speed) < ZERO_EPS:
        if robot.move_running:
            robot.start_continuous_move(forward_speed=0.0, side_speed=0.0, turn_speed=0.0)
        return

    if not robot.move_running:
        robot.enable_continuous_motion()
        time.sleep(_ENABLE_SETTLE_S)
    robot.start_continuous_move(
        forward_speed=forward_speed,
        side_speed=side_speed,
        turn_speed=turn_speed,
    )


def load_scenarios(path: Path) -> list[dict[str, Any]]:
    with path.open("r", encoding="utf-8") as handle:
        payload = json.load(handle)
    sequence = payload.get("sequence")
    if not isinstance(sequence, list):
        raise ValueError("Scenario file must contain a top-level 'sequence' list.")
    return sequence


def normalize_yolo_ws_uri(uri: str, token: str) -> str:
    """若 URI 未带 token，则追加 ?token= 或 &token=。"""
    if "token=" in uri:
        return uri
    return f"{uri}{'&' if '?' in uri else '?'}token={token}"


def fetch_frame_http(url: str, timeout: float = 2.0) -> FrameInput:
    req = urllib.request.Request(url, headers={"User-Agent": "track_live/1.0"})
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        raw = resp.read().decode("utf-8")
    payload = json.loads(raw)
    if "code" in payload:
        return FrameInput.from_http_response(payload)
    return FrameInput.from_dict(payload)


def infer_host_from_endpoint(endpoint: str | None) -> str | None:
    if not endpoint:
        return None
    try:
        parsed = urlparse(endpoint)
    except ValueError:
        return None
    host = parsed.hostname
    if not host:
        return None
    return host


def resolve_robot_ip(args: argparse.Namespace) -> str:
    if args.ip:
        return str(args.ip)

    inferred = infer_host_from_endpoint(args.yolo_ws) or infer_host_from_endpoint(args.yolo_url)
    if inferred:
        print(f"未显式提供 --ip，自动使用视觉源主机作为机器狗 IP: {inferred}")
        return inferred

    return DEFAULT_IP


def prepare_robot_for_tracking(robot: Controller) -> bool:
    """
    追踪专用初始化：不调用 Controller.initialize()（其中含自动模式，易覆盖起立姿态）。

    顺序与 udp_connect_stand 一致：心跳 → 手动 → STAND → 移动模式 → 再手动。
    持续运动：追踪中静止帧只发零轴，退出时在 main 的 finally 里 safe_stop 关通道。
    """
    print("\n" + "=" * 50)
    print("🤖 追踪：起立与模式（无自动模式）")
    print("=" * 50)
    try:
        print("\n[1/5] 启动心跳…")
        robot.start_heartbeat(frequency=2.0)
        time.sleep(1.0)
        print("\n[2/5] 切换到手动模式…")
        robot.switch_to_manual_mode()
        time.sleep(0.5)
        print("\n[3/5] 起立（语音 STAND）…")
        robot.voice_command("STAND")
        time.sleep(3.0)
        print("\n[4/5] 切换到移动模式…")
        robot.switch_to_move_mode()
        time.sleep(0.5)
        print("\n[5/5] 再次手动模式（UDP 连续轴控）…")
        robot.switch_to_manual_mode()
        time.sleep(0.3)
        print("\n" + "=" * 50)
        print("✓ 追踪就绪（请确认机器狗已站立）")
        print("=" * 50 + "\n")
        return True
    except Exception as e:
        print(f"\n❌ 追踪初始化失败: {e}")
        try:
            robot.stop_heartbeat()
        except OSError:
            pass
        return False


def apply_command(
    robot: Controller | None,
    cmd: TwistCommand,
    *,
    forward_gain: float = 2.0,
    forward_axis_min: float = _MIN_EFFECTIVE_FORWARD,
    forward_axis_max: float = 0.32,
    turn_gain: float = 1.0,
    turn_axis_min: float = _MIN_EFFECTIVE_TURN,
    turn_axis_max: float = 0.40,
    dry_run: bool = False,
) -> None:
    f, s, t = twist_to_move(
        cmd,
        forward_gain=forward_gain,
        forward_axis_min=forward_axis_min,
        forward_axis_max=forward_axis_max,
        turn_gain=turn_gain,
        turn_axis_min=turn_axis_min,
        turn_axis_max=turn_axis_max,
    )
    apply_motion_axes(
        robot,
        forward_speed=f,
        side_speed=s,
        turn_speed=t,
        dry_run=dry_run,
        cmd=cmd,
    )


def print_check_mapping(
    forward_gain: float = 2.0,
    forward_axis_min: float = _MIN_EFFECTIVE_FORWARD,
    forward_axis_max: float = 0.32,
    turn_gain: float = 1.0,
    turn_axis_min: float = 0.32,
    turn_axis_max: float = 0.40,
) -> None:
    """打印 linear_x/angular_z -> forward/turn 轴映射，供人工核对。"""
    rows: list[tuple[str, float, float]] = [
        ("stop", 0.0, 0.0),
        ("forward_only", 0.12, 0.0),
        ("forward_left", 0.12, 0.35),
        ("forward_right", 0.12, -0.35),
        ("small_forward", 0.02, 0.0),
    ]
    print("Twist linear_x/angular_z -> forward_speed/turn_speed")
    print(
        f"  forward_gain={forward_gain}  forward_axis_min={forward_axis_min}  forward_axis_max={forward_axis_max}"
    )
    print(f"  turn_gain={turn_gain}  turn_axis_min={turn_axis_min}  turn_axis_max={turn_axis_max}")
    print("-" * 72)
    for label, lx, az in rows:
        cmd = TwistCommand(
            linear_x=lx,
            angular_z=az,
            reason="check_mapping",
            state="TRACKING",
            previous_state="IDLE",
            lost_frame_count=0,
        )
        f, _s, t = twist_to_move(
            cmd,
            forward_gain=forward_gain,
            forward_axis_min=forward_axis_min,
            forward_axis_max=forward_axis_max,
            turn_gain=turn_gain,
            turn_axis_min=turn_axis_min,
            turn_axis_max=turn_axis_max,
        )
        print(
            f"  {label:22s}  linear_x={lx:+.2f} angular_z={az:+.2f}"
            f" -> forward={f:+.3f} turn={t:+.3f}"
        )


def run_smoke_udp(args: argparse.Namespace) -> None:
    """短动作烟测：前进 → 停 → 左转 → 右转，复用 udp_motion_demo.run_axis_motion。"""
    speed = args.smoke_speed
    duration = args.smoke_duration
    pause = args.smoke_pause
    print(f"UDP 烟测 ip={args.ip} port={args.port}  speed={speed} duration={duration}s")
    with SoftExitController((args.ip, args.port)) as robot:
        if not robot.initialize():
            print("initialize() 失败，退出", file=sys.stderr)
            sys.exit(1)
        try:
            run_axis_motion(robot, "烟测 前进", forward=speed, duration=duration, pause=pause)
            run_axis_motion(robot, "烟测 左转", turn=-speed, duration=duration, pause=pause)
            run_axis_motion(robot, "烟测 右转", turn=speed, duration=duration, pause=pause)
        finally:
            safe_stop(robot)
    print("烟测结束。")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="P4 Tracker 联机 UDP 控狗（简化版）")
    parser.add_argument(
        "--ip",
        default=None,
        help="机器狗 IP；若省略且使用 --yolo-ws/--yolo-url，则自动取对应 URL 的主机",
    )
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="机器狗 UDP 端口")
    parser.add_argument(
        "--check-mapping",
        action="store_true",
        help="只打印 Twist→轴映射表（不连狗），用于与 P4 文档对照",
    )
    parser.add_argument(
        "--smoke-udp",
        action="store_true",
        help="真机短动作烟测（前进/左转/右转），需与狗网络互通",
    )
    parser.add_argument(
        "--smoke-speed",
        type=float,
        default=0.25,
        help="--smoke-udp 时的轴速度幅度（与 udp_motion_demo 类似，约 0.2~0.4）",
    )
    parser.add_argument(
        "--smoke-duration",
        type=float,
        default=1.5,
        help="--smoke-udp 每段动作持续时间（秒）",
    )
    parser.add_argument(
        "--smoke-pause",
        type=float,
        default=0.5,
        help="--smoke-udp 每段动作之间的停顿（秒）",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="只打印映射结果，不连接机器狗",
    )
    # 现场调试优先调整：转向阈值、动作/冷却时长、前进增益和期望框高占比。
    parser.add_argument(
        "--forward-gain",
        type=float,
        default=2.5,
        help="前进轴：linear_x 乘该系数后再限幅；仅允许正向前进，不做后退",
    )
    parser.add_argument(
        "--forward-axis-min",
        type=float,
        default=_MIN_EFFECTIVE_FORWARD,
        help="最小有效前进轴幅度（需 >~0.20 才能跨越固件死区）",
    )
    parser.add_argument(
        "--forward-axis-max",
        type=float,
        default=0.35,
        help="最大前进轴幅度（限制不要冲得太猛）",
    )
    parser.add_argument(
        "--turn-gain",
        type=float,
        default=1.0,
        help="转向轴：(-angular_z) 乘该系数后再限幅",
    )
    parser.add_argument(
        "--turn-axis-min",
        type=float,
        default=0.30,
        help="最小有效转向轴幅度（需 >~0.29 才能跨越固件死区）",
    )
    parser.add_argument(
        "--turn-axis-max",
        type=float,
        default=0.32,
        help="最大转向轴幅度（限制不要转得过猛）",
    )
    parser.add_argument(
        "--desired-fill",
        type=float,
        default=0.85,
        help="目标框高占画面高度的期望比例；fill 小于该值时才前进（默认 0.85）",
    )
    parser.add_argument(
        "--far-start-boost",
        type=float,
        default=0.15,
        help="目标较远时额外抬高的 start_norm 上限增量，减少远距离小偏差导致的过度转向",
    )
    parser.add_argument(
        "--start-norm",
        type=float,
        default=0.30,
        help="触发转向动作的归一化偏差阈值 |norm|>=start 才转（建议 0.15~0.35）",
    )
    parser.add_argument(
        "--stop-norm",
        type=float,
        default=0.15,
        help="退出修正的归一化偏差阈值 |norm|<=stop 认为已对准（需 < start，用于迟滞）",
    )
    parser.add_argument(
        "--seen-n",
        type=int,
        default=4,
        help="连续看到目标 N 帧且满足偏差条件才触发一次动作（可抑制误检/抖动）",
    )
    parser.add_argument(
        "--action-s",
        type=float,
        default=0.10,
        metavar="SEC",
        help="一次转向动作持续时间（秒）。执行期间保持当前 turn 脉冲；前进仍按最新目标持续更新。",
    )
    parser.add_argument(
        "--cooldown-s",
        type=float,
        default=0.10,
        metavar="SEC",
        help="转向动作结束后的冷却时间（秒）。冷却期间 turn 保持 0，但前进仍可继续。",
    )
    src = parser.add_mutually_exclusive_group()
    src.add_argument(
        "--scenarios",
        type=Path,
        default=None,
        help="回放 scenarios.json（sequence[].frame），按 --delay 逐帧驱狗",
    )
    src.add_argument(
        "--yolo-url",
        default=None,
        help="轮询 YOLO HTTP，返回体符合 P4 §3（含 code）或 §4 内层 JSON",
    )
    src.add_argument(
        "--yolo-ws",
        default=None,
        help=f"YOLO WebSocket（YOLO客户端开发文档）：默认 {DEFAULT_YOLO_WS}，token 用 .env API_TOKEN",
    )
    parser.add_argument(
        "--delay",
        type=float,
        default=0.1,
        help="scenarios 模式下每帧间隔（秒）",
    )
    parser.add_argument(
        "--poll-hz",
        type=float,
        default=10.0,
        help="HTTP 模式下轮询频率（Hz）",
    )
    return parser.parse_args()


def run_scenarios(
    args: argparse.Namespace,
    tracker: P4TrackerController,
    robot: Controller | None,
) -> None:
    path = args.scenarios
    if path is None:
        path = Path("samples/scenarios.json")
    scenarios = load_scenarios(path)
    print(f"Loaded {len(scenarios)} frames from {path}")

    gate = TurnGateState()

    for index, scenario in enumerate(scenarios, start=1):
        try:
            frame = FrameInput.from_dict(scenario["frame"])
        except (KeyError, TypeError, ValueError) as e:
            print(f"Frame {index:02d} SKIP (invalid frame JSON: {e})")
            continue

        cmd = tracker.compute_command(frame)
        now = time.monotonic()
        fwd, side, turn = resolve_motion_axes(
            cmd,
            frame,
            gate,
            now=now,
            forward_gain=args.forward_gain,
            forward_axis_min=args.forward_axis_min,
            forward_axis_max=args.forward_axis_max,
            turn_gain=args.turn_gain,
            turn_axis_min=args.turn_axis_min,
            turn_axis_max=args.turn_axis_max,
            start_norm=float(args.start_norm),
            stop_norm=float(args.stop_norm),
            desired_fill=float(args.desired_fill),
            far_start_boost=float(args.far_start_boost),
            seen_n=int(args.seen_n),
            action_s=float(args.action_s),
            cooldown_s=float(args.cooldown_s),
        )
        name = scenario.get("name", "")
        print(
            f"Frame {index:02d} {name}  state {cmd.previous_state}->{cmd.state}  "
            f"linear_x={cmd.linear_x:.3f} angular_z={cmd.angular_z:.3f} "
            f"-> fwd={fwd:.3f} turn={turn:.3f}  {cmd.reason[:60]}..."
        )

        if robot is not None or args.dry_run:
            apply_motion_axes(
                robot,
                forward_speed=fwd,
                side_speed=side,
                turn_speed=turn,
                dry_run=args.dry_run,
                cmd=cmd,
            )

        time.sleep(args.delay)


def run_yolo_ws(
    args: argparse.Namespace,
    tracker: P4TrackerController,
    robot: Controller | None,
) -> None:
    """订阅机器狗 YOLO WebSocket，解析为 FrameInput 后走与 HTTP 相同的控制链。"""
    try:
        from dotenv import load_dotenv
    except ImportError:
        print("请安装: pip install python-dotenv websockets", file=sys.stderr)
        sys.exit(1)
    try:
        import websockets
    except ImportError:
        print("请安装: pip install websockets", file=sys.stderr)
        sys.exit(1)

    load_dotenv()
    token = os.getenv("API_TOKEN", "").strip()
    if not token and "token=" not in (args.yolo_ws or ""):
        print("请在 .env 设置 API_TOKEN，或在 --yolo-ws URL 中带 token=", file=sys.stderr)
        sys.exit(2)

    uri = normalize_yolo_ws_uri(args.yolo_ws, token) if token else args.yolo_ws
    print(f"YOLO WebSocket: {uri.split('?')[0]}?token=***")

    async def _loop() -> None:
        async with websockets.connect(uri, ping_interval=None) as ws:
            print("已连接，按 Ctrl+C 结束\n")
            gate = TurnGateState()
            async for raw in ws:
                msg = json.loads(raw)
                if msg.get("type") == "ping":
                    await ws.send(
                        json.dumps({"type": "pong", "ts": msg.get("ts")})
                    )
                    continue
                try:
                    frame = FrameInput.from_yolo_ws_payload(msg)
                except ValueError as e:
                    print(f"  [skip] {e}", file=sys.stderr)
                    continue

                now = time.monotonic()
                cmd = tracker.compute_command(frame)
                fwd, side, turn = resolve_motion_axes(
                    cmd,
                    frame,
                    gate,
                    now=now,
                    forward_gain=args.forward_gain,
                    forward_axis_min=args.forward_axis_min,
                    forward_axis_max=args.forward_axis_max,
                    turn_gain=args.turn_gain,
                    turn_axis_min=args.turn_axis_min,
                    turn_axis_max=args.turn_axis_max,
                    start_norm=float(args.start_norm),
                    stop_norm=float(args.stop_norm),
                    desired_fill=float(args.desired_fill),
                    far_start_boost=float(args.far_start_boost),
                    seen_n=int(args.seen_n),
                    action_s=float(args.action_s),
                    cooldown_s=float(args.cooldown_s),
                )
                offset_text = ""
                norm_text = ""
                fill_text = ""
                if cmd.target_metrics is not None:
                    iw = max(frame.image_width, 1)
                    offset = cmd.target_metrics.center_offset_x
                    norm = offset / (iw / 2.0)
                    fill = cmd.target_metrics.bbox_height / max(frame.image_height, 1)
                    offset_text = f" offset_px={offset:+.1f}"
                    norm_text = f" norm={norm:+.3f}"
                    fill_text = f" fill={fill:.2f}"

                print(
                    f"  state {cmd.previous_state}->{cmd.state}"
                    f"{offset_text}{norm_text}{fill_text}"
                    f"  linear_x={cmd.linear_x:+.3f} angular_z={cmd.angular_z:+.3f}"
                    f" -> fwd={fwd:+.3f} turn={turn:+.3f}"
                )

                if robot is not None:
                    apply_motion_axes(
                        robot,
                        forward_speed=fwd,
                        side_speed=side,
                        turn_speed=turn,
                        dry_run=False,
                    )

    try:
        asyncio.run(_loop())
    except KeyboardInterrupt:
        print("\n退出中…")


def run_http(
    args: argparse.Namespace,
    tracker: P4TrackerController,
    robot: Controller | None,
) -> None:
    assert args.yolo_url is not None
    interval = 1.0 / max(args.poll_hz, 0.1)
    print(f"Polling {args.yolo_url} at ~{args.poll_hz} Hz")

    try:
        gate = TurnGateState()
        while True:
            try:
                frame = fetch_frame_http(args.yolo_url)
            except (urllib.error.URLError, TimeoutError, json.JSONDecodeError, ValueError) as e:
                print(f"  HTTP/JSON error: {e}", file=sys.stderr)
                time.sleep(interval)
                continue

            now = time.monotonic()
            cmd = tracker.compute_command(frame)
            fwd, side, turn = resolve_motion_axes(
                cmd,
                frame,
                gate,
                now=now,
                forward_gain=args.forward_gain,
                forward_axis_min=args.forward_axis_min,
                forward_axis_max=args.forward_axis_max,
                turn_gain=args.turn_gain,
                turn_axis_min=args.turn_axis_min,
                turn_axis_max=args.turn_axis_max,
                start_norm=float(args.start_norm),
                stop_norm=float(args.stop_norm),
                desired_fill=float(args.desired_fill),
                far_start_boost=float(args.far_start_boost),
                seen_n=int(args.seen_n),
                action_s=float(args.action_s),
                cooldown_s=float(args.cooldown_s),
            )
            print(
                f"  state {cmd.previous_state}->{cmd.state}  "
                f"linear_x={cmd.linear_x:.3f} angular_z={cmd.angular_z:.3f} "
                f"-> fwd={fwd:.3f} turn={turn:.3f}"
            )

            if robot is not None:
                apply_motion_axes(
                    robot,
                    forward_speed=fwd,
                    side_speed=side,
                    turn_speed=turn,
                    dry_run=args.dry_run,
                )
            time.sleep(interval)
    except KeyboardInterrupt:
        print("\n退出中...")


def main() -> None:
    args = parse_args()
    args.ip = resolve_robot_ip(args)

    if args.check_mapping:
        print_check_mapping(
            forward_gain=args.forward_gain,
            forward_axis_min=args.forward_axis_min,
            forward_axis_max=args.forward_axis_max,
            turn_gain=args.turn_gain,
            turn_axis_min=args.turn_axis_min,
            turn_axis_max=args.turn_axis_max,
        )
        return

    if args.smoke_udp:
        run_smoke_udp(args)
        return

    if args.scenarios is None and args.yolo_url is None and args.yolo_ws is None:
        args.yolo_ws = DEFAULT_YOLO_WS

    tracker = P4TrackerController(desired_fill_ratio=float(args.desired_fill))

    if args.dry_run:
        print("Dry-run: 不初始化机器狗")
        if args.yolo_url:
            run_http(args, tracker, None)
        elif args.yolo_ws:
            run_yolo_ws(args, tracker, None)
        else:
            run_scenarios(args, tracker, None)
        return

    with SoftExitController((args.ip, args.port)) as robot:
        if not prepare_robot_for_tracking(robot):
            print("prepare_robot_for_tracking 失败，退出", file=sys.stderr)
            sys.exit(1)

        try:
            if args.yolo_url:
                run_http(args, tracker, robot)
            elif args.yolo_ws:
                run_yolo_ws(args, tracker, robot)
            else:
                run_scenarios(args, tracker, robot)
        finally:
            safe_stop(robot)


if __name__ == "__main__":
    main()
