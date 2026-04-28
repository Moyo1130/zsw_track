from __future__ import annotations

import time
from dataclasses import dataclass

from p4_tracker.controller import FrameInput, P4TrackerController, TwistCommand
from p4_tracker.controller import TRACKING_LOST
from totalController import Controller
from track_live import (
    TurnGateState,
    apply_motion_axes,
    prepare_robot_for_tracking,
    resolve_motion_axes,
)
from udp_motion_demo import safe_stop


@dataclass(frozen=True)
class TrackingConfig:
    # 现场调试优先调整：转向阈值、动作/冷却时长、前进增益和期望框高占比。
    forward_gain: float = 2.5
    forward_axis_min: float = 0.22
    forward_axis_max: float = 0.35
    turn_gain: float = 1.0
    turn_axis_min: float = 0.30
    turn_axis_max: float = 0.32
    start_norm: float = 0.30
    stop_norm: float = 0.15
    desired_fill: float = 0.85
    far_start_boost: float = 0.15
    seen_n: int = 4
    action_s: float = 0.10
    cooldown_s: float = 0.10


@dataclass(frozen=True)
class TrackingOutput:
    command: TwistCommand
    forward_speed: float
    side_speed: float
    turn_speed: float
    has_target: bool
    tracking_lost: bool
    selected_track_id: int | None


class TrackingManager:
    """状态机可调用的跟踪控制封装，内部复用 track_live.py 的现有控制链路。"""

    def __init__(
        self,
        *,
        robot: Controller | None = None,
        dry_run: bool = False,
        config: TrackingConfig | None = None,
        controller: P4TrackerController | None = None,
    ) -> None:
        self.robot = robot
        self.dry_run = dry_run
        self.config = config or TrackingConfig()
        self.controller = controller or P4TrackerController(
            desired_fill_ratio=self.config.desired_fill
        )
        self.gate = TurnGateState()
        self.active = False

    def start(self) -> bool:
        if self.active:
            return True
        if self.robot is not None and not self.dry_run:
            if not prepare_robot_for_tracking(self.robot):
                return False
        self.active = True
        return True

    def stop(self) -> None:
        if self.robot is not None and not self.dry_run:
            safe_stop(self.robot)
        self.active = False

    def reset(self) -> None:
        self.controller = P4TrackerController(
            desired_fill_ratio=self.config.desired_fill
        )
        self.gate = TurnGateState()

    def update(self, frame: FrameInput) -> TrackingOutput:
        cmd = self.controller.compute_command(frame)
        now = time.monotonic()
        fwd, side, turn = resolve_motion_axes(
            cmd,
            frame,
            self.gate,
            now=now,
            forward_gain=self.config.forward_gain,
            forward_axis_min=self.config.forward_axis_min,
            forward_axis_max=self.config.forward_axis_max,
            turn_gain=self.config.turn_gain,
            turn_axis_min=self.config.turn_axis_min,
            turn_axis_max=self.config.turn_axis_max,
            start_norm=self.config.start_norm,
            stop_norm=self.config.stop_norm,
            desired_fill=self.config.desired_fill,
            far_start_boost=self.config.far_start_boost,
            seen_n=self.config.seen_n,
            action_s=self.config.action_s,
            cooldown_s=self.config.cooldown_s,
        )
        output = TrackingOutput(
            command=cmd,
            forward_speed=fwd,
            side_speed=side,
            turn_speed=turn,
            has_target=cmd.selected_detection is not None,
            tracking_lost=cmd.state == TRACKING_LOST,
            selected_track_id=(
                None
                if cmd.selected_detection is None
                else cmd.selected_detection.track_id
            ),
        )
        self.apply(output)
        return output

    def apply(self, output: TrackingOutput) -> None:
        apply_motion_axes(
            self.robot,
            forward_speed=output.forward_speed,
            side_speed=output.side_speed,
            turn_speed=output.turn_speed,
            dry_run=self.dry_run,
            cmd=output.command,
        )
