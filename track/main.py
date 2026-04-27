from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any

from p4_tracker.controller import FrameInput, P4TrackerController, TwistCommand


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Replay sample YOLO frames and print the matching /cmd_vel decisions."
    )
    parser.add_argument(
        "--scenarios",
        type=Path,
        default=Path("samples/scenarios.json"),
        help="Path to the sample scenario JSON file.",
    )
    parser.add_argument(
        "--delay",
        type=float,
        default=0.0,
        help="Optional delay in seconds between frames. Defaults to 0.",
    )
    parser.add_argument(
        "--rate",
        type=int,
        default=10,
        help="Rate value used when rendering the equivalent rostopic command.",
    )
    return parser.parse_args()


def load_scenarios(path: Path) -> list[dict[str, Any]]:
    with path.open("r", encoding="utf-8") as handle:
        payload = json.load(handle)

    # 这里保持回放输入结构和未来真实适配层一致，
    # 这样后续只替换输入来源，不需要改控制器接口。
    sequence = payload.get("sequence")
    if not isinstance(sequence, list):
        raise ValueError("Scenario file must contain a top-level 'sequence' list.")
    return sequence


def print_decision(
    index: int,
    scenario: dict[str, Any],
    decision: TwistCommand,
    *,
    rate: int,
) -> None:
    print(f"Frame {index:02d} - {scenario['name']}")
    print(f"  Description : {scenario['description']}")
    print(f"  Timestamp   : {scenario['frame']['timestamp']}")
    print(f"  State       : {decision.previous_state} -> {decision.state}")
    print(f"  Reason      : {decision.reason}")
    print(f"  Lost Frames : {decision.lost_frame_count}")

    if decision.selected_detection is None:
        print("  Target      : none")
    else:
        detection = decision.selected_detection
        print(
            "  Target      : "
            f"class={detection.class_name}, score={detection.score:.2f}, "
            f"track_id={detection.track_id}"
        )

    if decision.target_metrics is None:
        print("  Metrics     : unavailable")
    else:
        metrics = decision.target_metrics
        print(
            "  Metrics     : "
            f"center_x={metrics.bbox_center_x:.1f}, "
            f"center_y={metrics.bbox_center_y:.1f}, "
            f"width={metrics.bbox_width:.1f}, "
            f"height={metrics.bbox_height:.1f}, "
            f"area={metrics.bbox_area:.1f}, "
            f"offset_x={metrics.center_offset_x:.1f}"
        )

    print(
        "  Command     : "
        f"linear_x={decision.linear_x:.2f}, angular_z={decision.angular_z:.2f}"
    )
    print("  rostopic    :")
    # 同时打印等价的 ROS 命令，方便本地联调结果和后续 /cmd_vel
    # 发布版本一一对应。
    for line in decision.to_rostopic_command(rate=rate).splitlines():
        print(f"    {line}")
    print()


def main() -> None:
    args = parse_args()
    scenarios = load_scenarios(args.scenarios)
    controller = P4TrackerController()

    print(f"Loaded {len(scenarios)} frames from {args.scenarios}")
    print()

    # 按顺序回放每一帧，便于观察状态切换和目标丢失后的恢复过程。
    for index, scenario in enumerate(scenarios, start=1):
        frame = FrameInput.from_dict(scenario["frame"])
        decision = controller.compute_command(frame)
        print_decision(index, scenario, decision, rate=args.rate)

        if args.delay > 0:
            time.sleep(args.delay)


if __name__ == "__main__":
    main()
