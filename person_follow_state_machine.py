from __future__ import annotations

import argparse
import asyncio
import json
import os
import sys
from dataclasses import asdict, dataclass, field
from enum import Enum
from importlib import import_module
from pathlib import Path
from typing import Any


TRACK_DIR = Path(__file__).resolve().parent / "track"
if str(TRACK_DIR) not in sys.path:
    sys.path.insert(0, str(TRACK_DIR))

from p4_tracker.controller import FrameInput  # noqa: E402
from totalController import Controller  # noqa: E402
from tracking_manager import TrackingManager, TrackingOutput  # noqa: E402
from track_live import DEFAULT_IP, DEFAULT_PORT, DEFAULT_YOLO_WS, normalize_yolo_ws_uri  # noqa: E402


def _patrol_api():
    return import_module("patrol_api")


def _websockets():
    return import_module("websockets")


class FollowState(str, Enum):
    PATROL = "PATROL"
    TRACKING_LOCK = "TRACKING_LOCK"
    TRACKING = "TRACKING"
    TRACKING_LOST = "TRACKING_LOST"
    RETURN_TO_PATROL = "RETURN_TO_PATROL"
    ERROR = "ERROR"


@dataclass(frozen=True)
class StateSnapshot:
    state: FollowState
    previous_state: FollowState
    locked_track_id: int | None = None
    lock_seen_frames: int = 0
    last_error: str | None = None
    patrol_context: dict[str, Any] = field(default_factory=dict)
    tracking: dict[str, Any] = field(default_factory=dict)


class PersonFollowStateMachine:
    """巡检中断追人状态机，直接复用 patrol_api.py 与 track/ 的现有代码。"""

    def __init__(
        self,
        *,
        tracking: TrackingManager,
        lock_confirm_frames: int = 1,
        pause_reason: str = "person_follow_interrupt",
    ) -> None:
        self.tracking = tracking
        self.lock_confirm_frames = max(1, int(lock_confirm_frames))
        self.pause_reason = pause_reason
        self.state = FollowState.PATROL
        self.previous_state = FollowState.PATROL
        self.lock_seen_frames = 0
        self.locked_track_id: int | None = None
        self.last_error: str | None = None
        self.patrol_context: dict[str, Any] = {}

    async def handle_frame(self, frame: FrameInput) -> StateSnapshot:
        try:
            if self.state == FollowState.PATROL:
                return await self._handle_patrol(frame)
            if self.state == FollowState.TRACKING_LOCK:
                return await self._handle_tracking_lock(frame)
            if self.state == FollowState.TRACKING:
                return await self._handle_tracking(frame)
            if self.state in {
                FollowState.TRACKING_LOST,
                FollowState.RETURN_TO_PATROL,
            }:
                return await self._return_to_patrol()
            return self._snapshot()
        except Exception as exc:
            self.last_error = str(exc)
            self._set_state(FollowState.ERROR)
            self.tracking.stop()
            return self._snapshot()

    async def stop(self, reason: str = "manual_stop") -> StateSnapshot:
        self.pause_reason = reason
        self.tracking.stop()
        if self.state not in {FollowState.PATROL, FollowState.ERROR}:
            await self._resume_patrol()
        self._set_state(FollowState.PATROL)
        self.lock_seen_frames = 0
        self.locked_track_id = None
        return self._snapshot()

    async def _handle_patrol(self, frame: FrameInput) -> StateSnapshot:
        if not self._has_person(frame):
            return self._snapshot()
        self.lock_seen_frames = 0
        self._set_state(FollowState.TRACKING_LOCK)
        return await self._handle_tracking_lock(frame)

    async def _handle_tracking_lock(self, frame: FrameInput) -> StateSnapshot:
        if not self._has_person(frame):
            self.lock_seen_frames = 0
            self.locked_track_id = None
            self._set_state(FollowState.PATROL)
            return self._snapshot()

        self.lock_seen_frames += 1
        if self.lock_seen_frames < self.lock_confirm_frames:
            return self._snapshot()

        await self._pause_patrol()
        if not self.tracking.start():
            raise RuntimeError("tracking manager start failed")
        output = self.tracking.update(frame)
        self.locked_track_id = output.selected_track_id
        self._set_state(FollowState.TRACKING)
        return self._snapshot(output)

    async def _handle_tracking(self, frame: FrameInput) -> StateSnapshot:
        output = self.tracking.update(frame)
        if output.selected_track_id is not None:
            self.locked_track_id = output.selected_track_id
        if output.tracking_lost:
            self._set_state(FollowState.TRACKING_LOST)
            return await self._return_to_patrol(output)
        return self._snapshot(output)

    async def _return_to_patrol(
        self,
        output: TrackingOutput | None = None,
    ) -> StateSnapshot:
        self._set_state(FollowState.RETURN_TO_PATROL)
        self.tracking.stop()
        self.tracking.reset()
        self.lock_seen_frames = 0
        self.locked_track_id = None
        await self._resume_patrol()
        self._set_state(FollowState.PATROL)
        return self._snapshot(output)

    async def _pause_patrol(self) -> None:
        patrol_api = _patrol_api()
        websockets = _websockets()
        self.patrol_context = dict(patrol_api.PATROL_RUNTIME_STATE)
        async with websockets.connect(patrol_api.WS_URL) as ws:
            status = await patrol_api.get_robot_status(ws)
            context = await patrol_api.get_robot_context(ws)
            resp = await patrol_api.pause_patrol_workflow(ws, self.pause_reason)
        self.patrol_context.update(
            {
                "robot_status": status,
                "robot_context": context,
                "pause_response": resp,
            }
        )
        patrol_api.PATROL_RUNTIME_STATE["last_response"] = {"pause_workflow": resp}

    async def _resume_patrol(self) -> None:
        patrol_api = _patrol_api()
        websockets = _websockets()
        async with websockets.connect(patrol_api.WS_URL) as ws:
            resp = await patrol_api.resume_patrol_workflow(ws)
        patrol_api.PATROL_RUNTIME_STATE["last_response"] = {"resume_workflow": resp}

    def _set_state(self, state: FollowState) -> None:
        if state == self.state:
            return
        self.previous_state = self.state
        self.state = state

    def _snapshot(self, output: TrackingOutput | None = None) -> StateSnapshot:
        tracking: dict[str, Any] = {}
        if output is not None:
            tracking = {
                "forward_speed": output.forward_speed,
                "side_speed": output.side_speed,
                "turn_speed": output.turn_speed,
                "has_target": output.has_target,
                "tracking_lost": output.tracking_lost,
                "selected_track_id": output.selected_track_id,
                "controller_state": output.command.state,
                "reason": output.command.reason,
            }
        return StateSnapshot(
            state=self.state,
            previous_state=self.previous_state,
            locked_track_id=self.locked_track_id,
            lock_seen_frames=self.lock_seen_frames,
            last_error=self.last_error,
            patrol_context=self.patrol_context,
            tracking=tracking,
        )

    @staticmethod
    def _has_person(frame: FrameInput) -> bool:
        return any(item.class_name == "person" for item in frame.detections)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="巡检中断追人状态机")
    parser.add_argument("--yolo-ws", default=DEFAULT_YOLO_WS)
    parser.add_argument("--ip", default=DEFAULT_IP)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


async def run_yolo_state_machine(args: argparse.Namespace) -> None:
    websockets = _websockets()
    token = os.getenv("API_TOKEN", "").strip()
    uri = normalize_yolo_ws_uri(args.yolo_ws, token) if token else args.yolo_ws
    robot = None if args.dry_run else Controller((args.ip, args.port))
    tracking = TrackingManager(robot=robot, dry_run=args.dry_run)
    machine = PersonFollowStateMachine(tracking=tracking)
    async with websockets.connect(uri, ping_interval=None) as ws:
        async for raw in ws:
            msg = json.loads(raw)
            if msg.get("type") == "ping":
                await ws.send(json.dumps({"type": "pong", "ts": msg.get("ts")}))
                continue
            frame = FrameInput.from_yolo_ws_payload(msg)
            snapshot = await machine.handle_frame(frame)
            print(json.dumps(asdict(snapshot), ensure_ascii=False))


if __name__ == "__main__":
    asyncio.run(run_yolo_state_machine(parse_args()))
