#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
用合成目标框序列仿真 P4TrackerController，打印每帧指令（Twist + 映射到 UDP 轴）。

不连机器狗、不依赖 YOLO。示例：
  python simulate_tracking.py
  python simulate_tracking.py --scene pan
  python simulate_tracking.py --scene all --rows 25

真机逐帧回放（与 YOLO 无关，用于逐项验证 UDP/转向/死区）：
  python simulate_tracking.py --robot --scene pan --ip 10.69.235.139
  python simulate_tracking.py --robot --scene center_left --ip 10.69.235.139
  python simulate_tracking.py --robot --scene center_right --ip 10.69.235.139
  python simulate_tracking.py --robot --scene away_center --ip 10.69.235.139
  python simulate_tracking.py --robot --scene all --confirm --pause-between 4
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass

from p4_tracker.controller import BBox, Detection, FrameInput, P4TrackerController
from track_live import (
    DEFAULT_IP,
    DEFAULT_PORT,
    apply_command,
    prepare_robot_for_tracking,
    twist_to_move,
)
from udp_motion_demo import SoftExitController, safe_stop

IW = 1280
IH = 720
DT_MS = 100  # 与控制器里 ~10Hz 假设一致


def person_frame(
    ts_ms: int,
    *,
    center_x: float,
    bbox_h: float,
    track_id: int = 1,
    score: float = 0.95,
    cy: float | None = None,
    aspect_wh: float = 0.48,
) -> FrameInput:
    """在固定分辨率下生成单人检测：框水平居中于 center_x，高度 bbox_h（像素）。"""
    cy = IH / 2.0 if cy is None else cy
    w = max(8.0, bbox_h * aspect_wh)
    x1 = center_x - w / 2.0
    x2 = center_x + w / 2.0
    y1 = cy - bbox_h / 2.0
    y2 = cy + bbox_h / 2.0
    det = Detection(
        class_name="person",
        score=score,
        bbox=BBox(x1=x1, y1=y1, x2=x2, y2=y2),
        track_id=track_id,
    )
    return FrameInput(
        timestamp=ts_ms,
        image_width=IW,
        image_height=IH,
        detections=[det],
    )


def empty_frame(ts_ms: int) -> FrameInput:
    return FrameInput(timestamp=ts_ms, image_width=IW, image_height=IH, detections=[])


@dataclass(frozen=True)
class Scenario:
    name: str
    description: str
    frames: list[FrameInput]


def scenario_pan_follow() -> Scenario:
    """目标从画面右侧匀速移到左侧（应出现明显转向指令）。"""
    frames: list[FrameInput] = []
    t0 = 1_700_000_000_000
    steps = 24
    for i in range(steps):
        # 900 -> 380
        cx = 900.0 + (380.0 - 900.0) * (i / max(1, steps - 1))
        frames.append(
            person_frame(
                t0 + i * DT_MS,
                center_x=cx,
                bbox_h=340.0,
                track_id=42,
            )
        )
    return Scenario(
        name="pan",
        description="单人 track_id=42，中心 x 从 900 移到 380（横穿画面）",
        frames=frames,
    )


def scenario_center_to_left() -> Scenario:
    """目标框中心从画面中线匀速移到左侧（全程在中心线以左渐远，转向符号应稳定一侧）。"""
    frames: list[FrameInput] = []
    t0 = 1_700_000_010_000
    steps = 22
    cx0 = IW / 2.0
    cx1 = 380.0
    for i in range(steps):
        cx = cx0 + (cx1 - cx0) * (i / max(1, steps - 1))
        frames.append(
            person_frame(
                t0 + i * DT_MS,
                center_x=cx,
                bbox_h=340.0,
                track_id=201,
            )
        )
    return Scenario(
        name="center_left",
        description="单人 track_id=201，框中心 x 从 640（中心）匀速移到 380（左侧）",
        frames=frames,
    )


def scenario_center_to_right() -> Scenario:
    """目标框中心从画面中线匀速移到右侧。"""
    frames: list[FrameInput] = []
    t0 = 1_700_000_020_000
    steps = 22
    cx0 = IW / 2.0
    cx1 = 900.0
    for i in range(steps):
        cx = cx0 + (cx1 - cx0) * (i / max(1, steps - 1))
        frames.append(
            person_frame(
                t0 + i * DT_MS,
                center_x=cx,
                bbox_h=340.0,
                track_id=202,
            )
        )
    return Scenario(
        name="center_right",
        description="单人 track_id=202，框中心 x 从 640（中心）匀速移到 900（右侧）",
        frames=frames,
    )


def scenario_away_center() -> Scenario:
    """
    目标始终在画面水平中线，框高度持续变小 → 用「占屏高度」模拟人从中距不断走远。
    控制器以 fill 为距离代理：远离时 desired_fill - fill 增大，应以前进（linear_x）为主、角速度接近 0。
    """
    frames: list[FrameInput] = []
    t0 = 1_700_000_030_000
    steps = 28
    h_start = 420.0
    h_end = 120.0
    for i in range(steps):
        h = h_start + (h_end - h_start) * (i / max(1, steps - 1))
        frames.append(
            person_frame(
                t0 + i * DT_MS,
                center_x=IW / 2.0,
                bbox_h=h,
                track_id=303,
            )
        )
    return Scenario(
        name="away_center",
        description="单人 track_id=303，中心 x=640 不变，框高 420→120（持续走远，期望前进跟随）",
        frames=frames,
    )


def scenario_approach() -> Scenario:
    """框变高：模拟人走近（fill 增大，应减速/停下）。"""
    frames: list[FrameInput] = []
    t0 = 1_700_000_000_000
    steps = 20
    for i in range(steps):
        h = 220.0 + (520.0 - 220.0) * (i / max(1, steps - 1))
        frames.append(
            person_frame(
                t0 + i * DT_MS,
                center_x=640.0,
                bbox_h=h,
                track_id=3,
            )
        )
    return Scenario(
        name="approach",
        description="目标始终在画面中心，框高度 220→520（走近）",
        frames=frames,
    )


def scenario_recede() -> Scenario:
    """框变矮：模拟人走远（应更倾向于前进）。"""
    frames: list[FrameInput] = []
    t0 = 1_700_000_000_000
    steps = 18
    for i in range(steps):
        h = 480.0 + (200.0 - 480.0) * (i / max(1, steps - 1))
        frames.append(
            person_frame(
                t0 + i * DT_MS,
                center_x=640.0,
                bbox_h=h,
                track_id=3,
            )
        )
    return Scenario(
        name="recede",
        description="目标始终在画面中心，框高度 480→200（走远）",
        frames=frames,
    )


def scenario_lost_and_relock() -> Scenario:
    """锁定 id=5 后连续丢检测，超过 lost_timeout 解锁；再出现新人 id=7。"""
    frames: list[FrameInput] = []
    t0 = 1_700_000_000_000
    k = 0
    # 先稳定跟几帧
    for _ in range(4):
        frames.append(person_frame(t0 + k * DT_MS, center_x=620.0, bbox_h=300.0, track_id=5))
        k += 1
    # 丢 12 帧（默认 lost_timeout=10 会进入 TRACKING_LOST 并清锁）
    for _ in range(12):
        frames.append(empty_frame(t0 + k * DT_MS))
        k += 1
    # 新人出现（高置信度）
    for _ in range(6):
        frames.append(person_frame(t0 + k * DT_MS, center_x=700.0, bbox_h=310.0, track_id=7))
        k += 1
    return Scenario(
        name="lost",
        description="先跟 track_id=5，再空 12 帧，再出现 track_id=7（观察 BUFFER/LOST/重锁）",
        frames=frames,
    )


def scenario_jitter() -> Scenario:
    """中心小幅抖动 + 框高微动，观察平滑后指令。"""
    import random

    rng = random.Random(0)
    frames: list[FrameInput] = []
    t0 = 1_700_000_000_000
    for i in range(30):
        cx = 640.0 + rng.uniform(-80.0, 80.0)
        h = 300.0 + rng.uniform(-25.0, 25.0)
        frames.append(
            person_frame(t0 + i * DT_MS, center_x=cx, bbox_h=h, track_id=99),
        )
    return Scenario(
        name="jitter",
        description="中心 ±80px、高度 ±25px 随机抖动（固定 track_id）",
        frames=frames,
    )


ALL_SCENARIOS: list[Scenario] = [
    scenario_pan_follow(),
    scenario_center_to_left(),
    scenario_center_to_right(),
    scenario_away_center(),
    scenario_approach(),
    scenario_recede(),
    scenario_lost_and_relock(),
    scenario_jitter(),
]


def print_run(
    scenario: Scenario,
    *,
    max_rows: int | None,
    forward_gain: float,
    turn_gain: float,
) -> None:
    ctrl = P4TrackerController()
    print()
    print("=" * 88)
    print(f"[{scenario.name}] {scenario.description}")
    print("=" * 88)

    header = (
        f"{'#':>3}  {'state':^14}  {'lin_x':>7}  {'ang_z':>7}  "
        f"{'fwd':>7}  {'turn':>7}  offset_px  fill   lost  reason"
    )
    print(header)
    print("-" * len(header))

    for i, frame in enumerate(scenario.frames):
        if max_rows is not None and i >= max_rows:
            print(f"... 截断，仅显示前 {max_rows} 行（共 {len(scenario.frames)} 帧）")
            break

        cmd = ctrl.compute_command(frame)
        f_spd, _s, t_spd = twist_to_move(
            cmd, forward_gain=forward_gain, turn_gain=turn_gain
        )
        off = ""
        fill = ""
        if cmd.target_metrics is not None:
            off = f"{cmd.target_metrics.center_offset_x:>9.1f}"
            fill = f"{cmd.target_metrics.bbox_height / IH:>5.2f}"
        else:
            off = "      —"
            fill = "   —"

        reason = cmd.reason.replace("\n", " ")
        if len(reason) > 42:
            reason = reason[:39] + "..."

        print(
            f"{i:3d}  {cmd.state:^14}  {cmd.linear_x:7.3f}  {cmd.angular_z:7.3f}  "
            f"{f_spd:7.3f}  {t_spd:7.3f}  {off}  {fill}  {cmd.lost_frame_count:4d}  {reason}"
        )


def _zero_hold(robot: SoftExitController, duration_s: float) -> None:
    """场景间隙发零速，避免上一段角速度带到下一段。"""
    if duration_s <= 0:
        return
    if robot.move_running:
        robot.start_continuous_move(forward_speed=0.0, side_speed=0.0, turn_speed=0.0)
    time.sleep(duration_s)


def run_scenario_robot(
    robot: SoftExitController,
    scenario: Scenario,
    *,
    max_rows: int | None,
    forward_gain: float,
    turn_gain: float,
    dt_s: float,
    quiet: bool,
    log_every: int,
    verbose_table: bool,
) -> None:
    """逐帧把合成检测送入控制器并发 UDP（与 track_live 相同映射）。"""
    ctrl = P4TrackerController()
    total = len(scenario.frames) if max_rows is None else min(len(scenario.frames), max_rows)

    if not quiet:
        print()
        print("=" * 72)
        print(f"真机回放  [{scenario.name}]  {scenario.description}")
        print(f"帧数={total}  间隔={dt_s:.3f}s  forward_gain={forward_gain}  turn_gain={turn_gain}")
        print("=" * 72)

    if verbose_table and not quiet:
        header = (
            f"{'#':>3}  {'state':^14}  {'lin_x':>7}  {'ang_z':>7}  "
            f"{'fwd':>7}  {'turn':>7}  offset_px  fill   lost  reason"
        )
        print(header)
        print("-" * len(header))

    for i, frame in enumerate(scenario.frames):
        if max_rows is not None and i >= max_rows:
            break
        cmd = ctrl.compute_command(frame)
        f_spd, _s, t_spd = twist_to_move(
            cmd, forward_gain=forward_gain, turn_gain=turn_gain
        )

        do_print = not quiet and (log_every <= 1 or i % log_every == 0 or i == total - 1)
        if do_print:
            if verbose_table:
                off = ""
                fill = ""
                if cmd.target_metrics is not None:
                    off = f"{cmd.target_metrics.center_offset_x:>9.1f}"
                    fill = f"{cmd.target_metrics.bbox_height / IH:>5.2f}"
                else:
                    off = "      —"
                    fill = "   —"
                reason = cmd.reason.replace("\n", " ")
                if len(reason) > 42:
                    reason = reason[:39] + "..."
                print(
                    f"{i:3d}  {cmd.state:^14}  {cmd.linear_x:7.3f}  {cmd.angular_z:7.3f}  "
                    f"{f_spd:7.3f}  {t_spd:7.3f}  {off}  {fill}  {cmd.lost_frame_count:4d}  {reason}"
                )
            else:
                print(
                    f"  [{i:3d}/{total}]  {cmd.state:16s}  "
                    f"fwd={f_spd:+.3f} turn={t_spd:+.3f}  "
                    f"lx={cmd.linear_x:+.3f} az={cmd.angular_z:+.3f}"
                )

        apply_command(
            robot,
            cmd,
            forward_gain=forward_gain,
            turn_gain=turn_gain,
            dry_run=False,
        )
        time.sleep(dt_s)

    if not quiet:
        print(f"--- 场景 [{scenario.name}] 结束，已发零速前等待 ---")
    _zero_hold(robot, 0.25)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="合成目标框仿真 Tracker 输出")
    p.add_argument(
        "--scene",
        choices=[
            "all",
            "pan",
            "center_left",
            "center_right",
            "away_center",
            "approach",
            "recede",
            "lost",
            "jitter",
        ],
        default="all",
        help="要运行的场景；all 为全部依次运行",
    )
    p.add_argument(
        "--rows",
        type=int,
        default=None,
        metavar="N",
        help="每个场景最多执行/打印前 N 帧（默认不截断）",
    )
    p.add_argument("--forward-gain", type=float, default=1.0)
    p.add_argument("--turn-gain", type=float, default=1.0)

    p.add_argument(
        "--robot",
        action="store_true",
        help="连接机器狗 UDP，按合成帧逐帧下发连续轴（需空地、人已站立）",
    )
    p.add_argument("--ip", default=DEFAULT_IP, help="机器狗 IP（仅 --robot）")
    p.add_argument("--port", type=int, default=DEFAULT_PORT, help="UDP 端口（仅 --robot）")
    p.add_argument(
        "--dt",
        type=float,
        default=DT_MS / 1000.0,
        help="真机模式下每帧间隔（秒），默认与合成时间步一致",
    )
    p.add_argument(
        "--pause-between",
        type=float,
        default=2.0,
        metavar="SEC",
        help="--robot 且 --scene all 时，场景之间额外停顿（秒）并发零速",
    )
    p.add_argument(
        "--confirm",
        action="store_true",
        help="--robot 且多场景时，每段开始前按 Enter 继续",
    )
    p.add_argument(
        "--quiet",
        action="store_true",
        help="真机模式几乎不打日志，仅场景名与结束提示",
    )
    p.add_argument(
        "--verbose-table",
        action="store_true",
        help="真机模式也打印与纯仿真相同宽表（默认为一行精简日志）",
    )
    p.add_argument(
        "--log-every",
        type=int,
        default=1,
        metavar="K",
        help="真机精简模式下每 K 帧打印一行（末帧始终打印）",
    )
    return p.parse_args()


def main() -> None:
    args = parse_args()
    name_map = {s.name: s for s in ALL_SCENARIOS}

    if args.robot:
        if args.dt <= 0:
            print("--dt 必须 > 0", file=sys.stderr)
            sys.exit(2)
        if args.log_every < 1:
            print("--log-every 必须 >= 1", file=sys.stderr)
            sys.exit(2)

        scenarios_list: list[Scenario]
        if args.scene == "all":
            scenarios_list = list(ALL_SCENARIOS)
        else:
            scenarios_list = [name_map[args.scene]]

        print(
            "\n即将用「合成 bbox」驱动真机：与摄像头无关，仅验证控制器→UDP 映射。"
            "\n请确认周围无障碍、狗已可站立行走，按 Ctrl+C 可随时中断。\n"
        )

        with SoftExitController((args.ip, args.port)) as robot:
            if not prepare_robot_for_tracking(robot):
                print("prepare_robot_for_tracking 失败", file=sys.stderr)
                sys.exit(1)
            try:
                for idx, s in enumerate(scenarios_list):
                    if args.confirm:
                        if idx == 0:
                            input(f"按 Enter 开始场景 [{s.name}] …")
                        else:
                            input(f"按 Enter 开始下一场景 [{s.name}] …")

                    run_scenario_robot(
                        robot,
                        s,
                        max_rows=args.rows,
                        forward_gain=args.forward_gain,
                        turn_gain=args.turn_gain,
                        dt_s=args.dt,
                        quiet=args.quiet,
                        log_every=args.log_every,
                        verbose_table=args.verbose_table,
                    )

                    if idx < len(scenarios_list) - 1 and args.pause_between > 0:
                        if not args.confirm and not args.quiet:
                            print(
                                f"场景间停顿 {args.pause_between:.1f}s（零速）…"
                            )
                        _zero_hold(robot, args.pause_between)
            except KeyboardInterrupt:
                print("\n用户中断，刹停…", file=sys.stderr)
            finally:
                safe_stop(robot)

        print("\n真机序列结束。")
        return

    if args.scene == "all":
        for s in ALL_SCENARIOS:
            print_run(
                s,
                max_rows=args.rows,
                forward_gain=args.forward_gain,
                turn_gain=args.turn_gain,
            )
    else:
        print_run(
            name_map[args.scene],
            max_rows=args.rows,
            forward_gain=args.forward_gain,
            turn_gain=args.turn_gain,
        )

    print()
    print(
        "列说明：lin_x/angular_z 为控制器 Twist；fwd/turn 为经 twist_to_move 后的 UDP 连续轴（含死区抬升）。"
    )
    print(f"画面尺寸固定为 {IW}x{IH}，时间步约 {DT_MS}ms/帧。")
    print("真机逐项测试请加 --robot（见文件顶部 docstring）。")
    print()


if __name__ == "__main__":
    main()
