#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
单帧 JSON → P4TrackerController → UDP 驱狗（短时一次动作）

用于一条一条验证：改一个 JSON 文件，跑一次，观察狗是否对应动一下。

JSON 支持两种顶层结构：
  1) 与 YOLO/检出一帧一致：含 timestamp、image_width、image_height、detections
  2) 含 wrapper：{ "frame": { ... 同上 ... } }
  3) HTTP 封装：{ "code": 0, "message": "ok", ... 同上字段 }

用法：
  python track_json_one.py samples/frames/turn_left.json --dry-run
  python track_json_one.py samples/frames/forward.json --duration 1.5
  # 狗已站立、只想测动作：加 --quick（不重复起立）
  python track_json_one.py samples/frames/forward.json --duration 1.5 --quick
  python track_json_one.py --stdin --dry-run
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any

from p4_tracker.controller import FrameInput, P4TrackerController
from totalController import Controller
from track_live import (
    DEFAULT_IP,
    DEFAULT_PORT,
    is_stationary,
    prepare_robot_for_tracking,
    twist_to_move,
)
from udp_motion_demo import SoftExitController, safe_stop


def prepare_robot_quick(robot: Controller) -> bool:
    """
    不发起立语音、不切自动模式：适合狗已站立时测单帧动作。
    避免重复 STAND 在部分固件上触发趴→站动画，盖住后续前进。
    """
    try:
        robot.start_heartbeat(frequency=2.0)
        time.sleep(0.8)
        robot.switch_to_move_mode()
        time.sleep(0.5)
        robot.switch_to_manual_mode()
        time.sleep(0.4)
        return True
    except Exception as e:
        print(f"prepare_robot_quick 失败: {e}", file=sys.stderr)
        return False


def load_json_data(path: Path | None, stdin: bool) -> dict[str, Any]:
    if stdin:
        raw = sys.stdin.read()
        if not raw.strip():
            raise ValueError("标准输入为空")
        return json.loads(raw)
    if path is None:
        raise ValueError("请指定 JSON 文件或使用 --stdin")
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def payload_to_frame(data: dict[str, Any]) -> FrameInput:
    if data.get("code") is not None:
        return FrameInput.from_http_response(data)
    inner = data.get("frame")
    if isinstance(inner, dict):
        return FrameInput.from_dict(inner)
    if "timestamp" in data and "detections" in data:
        return FrameInput.from_dict(data)
    raise ValueError(
        "无法解析为单帧：需要含 frame，或为整帧字段（timestamp + detections），"
        "或为 HTTP 格式（含 code）"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="读取单帧 JSON，经 Tracker 解算后对机器狗执行一次短时动作",
    )
    parser.add_argument(
        "json_file",
        type=Path,
        nargs="?",
        default=None,
        help="单帧 JSON 文件路径（与 --stdin 二选一）",
    )
    parser.add_argument(
        "--stdin",
        action="store_true",
        help="从标准输入读取 JSON",
    )
    parser.add_argument("--ip", default=DEFAULT_IP, help="机器狗 IP")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="UDP 端口")
    parser.add_argument(
        "--duration",
        type=float,
        default=1.5,
        help="非静止指令时持续 send 连续运动的时长（秒）",
    )
    parser.add_argument(
        "--forward-gain",
        type=float,
        default=1.0,
        help="linear_x 乘系数后限幅",
    )
    parser.add_argument(
        "--turn-gain",
        type=float,
        default=1.0,
        help="(-angular_z) 乘系数后限幅",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="只打印解算结果与轴映射，不连接机器狗",
    )
    parser.add_argument(
        "--quick",
        action="store_true",
        help="狗已站立：仅心跳+移动+手动，不发起立（与 track_live 完整准备互斥）",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.stdin and args.json_file is not None:
        print("请只指定 --stdin 或文件路径其一", file=sys.stderr)
        sys.exit(2)

    data = load_json_data(args.json_file, args.stdin)
    frame = payload_to_frame(data)
    tracker = P4TrackerController()
    cmd = tracker.compute_command(frame)

    f, s, t = twist_to_move(
        cmd,
        forward_gain=args.forward_gain,
        turn_gain=args.turn_gain,
    )

    print("=== 解算结果 ===")
    print(f"  state      : {cmd.previous_state} -> {cmd.state}")
    print(f"  reason     : {cmd.reason}")
    print(f"  twist      : linear_x={cmd.linear_x:.3f}  angular_z={cmd.angular_z:.3f}")
    print(f"  -> move    : forward={f:.3f}  side={s:.3f}  turn={t:.3f}")
    print(f"  stationary : {is_stationary(cmd)}")

    if args.dry_run:
        print("（--dry-run 结束，未发 UDP）")
        return

    duration = max(0.05, args.duration)
    with SoftExitController((args.ip, args.port)) as robot:
        if args.quick:
            if not prepare_robot_quick(robot):
                sys.exit(1)
        else:
            if not prepare_robot_for_tracking(robot):
                print("prepare_robot_for_tracking 失败", file=sys.stderr)
                sys.exit(1)
        try:
            if is_stationary(cmd):
                print("静止指令：执行 safe_stop…")
                safe_stop(robot)
                return
            print(f"开始连续运动 {duration:.2f}s，请观察狗的动作…")
            if not robot.move_running:
                robot.enable_continuous_motion()
                time.sleep(0.15)
            robot.start_continuous_move(forward_speed=f, side_speed=s, turn_speed=t)
            time.sleep(duration)
        finally:
            print("刹停：停线程、零轴、关闭持续运动通道…")
            safe_stop(robot)
            print("已停止连续运动。")


if __name__ == "__main__":
    main()
