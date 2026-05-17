from __future__ import annotations

import argparse
import asyncio
import json
import os
import re
import sys
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime
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


def _load_env_token() -> str:
    try:
        from dotenv import load_dotenv
    except ImportError:
        return os.getenv("API_TOKEN", "").strip()

    root_env = Path(__file__).resolve().parent / ".env"
    track_env = TRACK_DIR / ".env"
    load_dotenv(root_env)
    load_dotenv(track_env, override=False)
    return os.getenv("API_TOKEN", "").strip()


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
    detection_debug: dict[str, Any] = field(default_factory=dict)
    timing: dict[str, Any] = field(default_factory=dict)


class PersonFollowStateMachine:
    """巡检中断追人状态机，直接复用 patrol_api.py 与 track/ 的现有代码。"""

    def __init__(
        self,
        *,
        tracking: TrackingManager,
        lock_confirm_frames: int = 5,
        min_person_score: float = 0.70,
        min_person_height_ratio: float = 0.15,
        pause_reason: str = "person_follow_interrupt",
        map_name: str | None = None,
        route_name: str | None = None,
        rebuild_on_bad_resume: bool = True,
    ) -> None:
        self.tracking = tracking
        self.lock_confirm_frames = max(1, int(lock_confirm_frames))
        self.min_person_score = float(min_person_score)
        self.min_person_height_ratio = float(min_person_height_ratio)
        self.pause_reason = pause_reason
        self.map_name = map_name
        self.route_name = route_name
        self.rebuild_on_bad_resume = rebuild_on_bad_resume
        self.state = FollowState.PATROL
        self.previous_state = FollowState.PATROL
        self.lock_seen_frames = 0
        self.locked_track_id: int | None = None
        self.pending_track_id: int | None = None
        self.last_error: str | None = None
        self.patrol_context: dict[str, Any] = {}
        self.detection_debug: dict[str, Any] = {}
        self._return_cleanup_done = False
        self._resume_attempt_count = 0
        self._run_started_monotonic = time.monotonic()
        self._run_started_wall_iso = self._now_iso()
        self._state_entered_monotonic = self._run_started_monotonic
        self._state_entered_wall_iso = self._run_started_wall_iso
        self._state_sequence = 0
        self._last_transition: dict[str, Any] | None = None

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
        candidate = self._select_lock_candidate(frame)
        if candidate is None:
            return self._snapshot()
        self.pending_track_id = candidate.track_id
        self.lock_seen_frames = 0
        self._set_state(FollowState.TRACKING_LOCK)
        return await self._handle_tracking_lock(frame)

    async def _handle_tracking_lock(self, frame: FrameInput) -> StateSnapshot:
        candidate = self._select_lock_candidate(frame)
        if candidate is None:
            self.lock_seen_frames = 0
            self.pending_track_id = None
            self.locked_track_id = None
            self._set_state(FollowState.PATROL)
            return self._snapshot()

        if (
            self.pending_track_id is not None
            and candidate.track_id is not None
            and candidate.track_id != self.pending_track_id
        ):
            self.pending_track_id = candidate.track_id
            self.lock_seen_frames = 1
        else:
            self.lock_seen_frames += 1

        if self.lock_seen_frames < self.lock_confirm_frames:
            return self._snapshot()

        await self._pause_patrol()
        if not self.tracking.start():
            raise RuntimeError("tracking manager start failed")
        self._return_cleanup_done = False
        self._resume_attempt_count = 0
        self.last_error = None
        output = self.tracking.update(frame)
        self.locked_track_id = output.selected_track_id
        self.pending_track_id = None
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
        if not self._return_cleanup_done:
            self.tracking.stop(restore_auto=True)
            self.tracking.reset()
            self._return_cleanup_done = True
        self.lock_seen_frames = 0
        self.locked_track_id = None
        self.pending_track_id = None

        try:
            await self._resume_patrol()
        except Exception as exc:
            self.last_error = str(exc)
            return self._snapshot(output)

        self._return_cleanup_done = False
        self._resume_attempt_count = 0
        self.last_error = None
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
                "resume_plan": self._build_resume_plan(context),
            }
        )
        patrol_api.PATROL_RUNTIME_STATE["last_response"] = {"pause_workflow": resp}

    async def _resume_patrol(self) -> None:
        patrol_api = _patrol_api()
        websockets = _websockets()
        last_error: Exception | None = None

        for attempt in range(1, 6):
            self._resume_attempt_count += 1
            try:
                await asyncio.sleep(0.5 * attempt)
                async with websockets.connect(patrol_api.WS_URL) as ws:
                    auto_resp = await patrol_api.switch_to_auto_mode(ws)
                    await asyncio.sleep(0.5)
                    resp = await patrol_api.resume_patrol_workflow(ws)
                    await asyncio.sleep(0.5)
                    status = await patrol_api.get_robot_status(ws)
                    context = await patrol_api.get_robot_context(ws)
                    resume_already_running = False
                    if resp.get("success") == 0:
                        resume_already_running = self._is_patrol_running(
                            status,
                            context,
                        )
                        if not resume_already_running:
                            raise RuntimeError(
                                resp.get("message", f"resume failed: {resp}")
                            )
                    restart_resp = None
                    restart_status = None
                    restart_context = None
                    if (
                        self.rebuild_on_bad_resume
                        and self._should_rebuild_after_resume(status, context)
                    ):
                        restart_resp = await patrol_api.restart_patrol_from_waypoint(
                            ws,
                            self._resolve_resume_map_name(),
                            self._resolve_resume_route_name(context),
                            self._resolve_resume_start_index(),
                        )
                        await asyncio.sleep(0.5)
                        restart_status = await patrol_api.get_robot_status(ws)
                        restart_context = await patrol_api.get_robot_context(ws)
                resume_context = {
                    "auto_mode": auto_resp,
                    "resume_workflow": resp,
                    "resume_already_running": resume_already_running,
                    "post_resume_status": status,
                    "post_resume_context": context,
                    "restart_from_waypoint": restart_resp,
                    "post_restart_status": restart_status,
                    "post_restart_context": restart_context,
                }
                patrol_api.PATROL_RUNTIME_STATE["last_response"] = {
                    "resume_workflow": resp,
                    "resume_already_running": resume_already_running,
                    "auto_mode": auto_resp,
                    "post_resume_status": status,
                    "post_resume_context": context,
                    "restart_from_waypoint": restart_resp,
                    "post_restart_status": restart_status,
                    "post_restart_context": restart_context,
                }
                patrol_api.PATROL_RUNTIME_STATE["mode"] = "running"
                self.patrol_context.update(resume_context)
                return
            except Exception as exc:
                last_error = exc
                self.last_error = (
                    f"resume retry {attempt}/5 failed "
                    f"(total={self._resume_attempt_count}): {exc}"
                )

        raise RuntimeError(
            f"resume patrol failed after retries: {last_error}"
        )

    @staticmethod
    def _is_patrol_running(
        status: dict[str, Any],
        context: dict[str, Any],
    ) -> bool:
        status_data = status.get("data") or {}
        context_data = context.get("data") or {}
        if not status_data.get("executing"):
            return False
        if status_data.get("status") != 1:
            return False
        if status_data.get("inspection_state") != 1:
            return False
        return bool(
            context_data.get("workflow_name")
            or context_data.get("route_name")
            or context_data.get("current_waypoint_name")
        )

    @staticmethod
    def _int_or_none(value: Any) -> int | None:
        try:
            if value is None or value == "":
                return None
            return int(value)
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _route_index_from_waypoint_name(value: Any) -> int | None:
        if value is None:
            return None
        match = re.fullmatch(r"point_(\d+)", str(value).strip(), re.IGNORECASE)
        if not match:
            return None
        return int(match.group(1))

    def _build_resume_plan(self, context: dict[str, Any]) -> dict[str, Any]:
        data = context.get("data") or {}
        current_index = self._int_or_none(data.get("current_waypoint_index"))
        last_index = self._int_or_none(data.get("last_arrived_waypoint_index"))
        current_route_index = (
            self._route_index_from_waypoint_name(data.get("current_waypoint_name"))
            or current_index
        )
        last_route_index = (
            self._route_index_from_waypoint_name(data.get("last_arrived_waypoint_name"))
            or last_index
        )
        if current_route_index is not None and (
            last_route_index is None or current_route_index > last_route_index
        ):
            start_index = current_route_index
        elif last_route_index is not None:
            start_index = last_route_index + 1
        else:
            start_index = current_route_index or 1

        return {
            "map_name": self.map_name,
            "route_name": (
                self.route_name
                or self.patrol_context.get("route_name")
                or self.patrol_context.get("original_route_name")
                or data.get("route_name")
            ),
            "workflow_name": data.get("workflow_name") or data.get("tree_name"),
            "start_index": max(1, start_index),
            "pause_current_waypoint_index": current_index,
            "pause_current_waypoint_name": data.get("current_waypoint_name"),
            "pause_current_route_index": current_route_index,
            "pause_last_arrived_waypoint_index": last_index,
            "pause_last_arrived_waypoint_name": data.get("last_arrived_waypoint_name"),
            "pause_last_arrived_route_index": last_route_index,
        }

    def _resolve_resume_map_name(self) -> str:
        plan = self.patrol_context.get("resume_plan") or {}
        map_name = (
            self.map_name
            or plan.get("map_name")
            or self.patrol_context.get("map_name")
        )
        if not map_name:
            raise RuntimeError("map_name is required for patrol workflow rebuild")
        return str(map_name)

    def _resolve_resume_route_name(self, context: dict[str, Any]) -> str:
        plan = self.patrol_context.get("resume_plan") or {}
        data = context.get("data") or {}
        route_name = (
            self.route_name
            or plan.get("route_name")
            or self.patrol_context.get("route_name")
            or self.patrol_context.get("original_route_name")
            or data.get("route_name")
        )
        if not route_name:
            raise RuntimeError("route_name is required for patrol workflow rebuild")
        return str(route_name)

    def _resolve_resume_start_index(self) -> int:
        plan = self.patrol_context.get("resume_plan") or {}
        start_index = self._int_or_none(plan.get("start_index"))
        return max(1, start_index or 1)

    def _should_rebuild_after_resume(
        self,
        status: dict[str, Any],
        context: dict[str, Any],
    ) -> bool:
        data = context.get("data") or {}
        status_data = status.get("data") or {}
        current_index = self._int_or_none(data.get("current_waypoint_index"))
        last_index = self._int_or_none(data.get("last_arrived_waypoint_index"))
        current_route_index = (
            self._route_index_from_waypoint_name(data.get("current_waypoint_name"))
            or current_index
        )
        last_route_index = (
            self._route_index_from_waypoint_name(data.get("last_arrived_waypoint_name"))
            or last_index
        )
        expected_index = self._resolve_resume_start_index()

        if current_route_index is not None and current_route_index < expected_index:
            return True
        if (
            current_route_index is not None
            and last_route_index is not None
            and last_route_index > 0
            and current_route_index <= last_route_index
        ):
            return True

        tasks = status_data.get("tasks")
        running_tasks = []
        if isinstance(tasks, list):
            running_tasks = [
                item
                for item in tasks
                if isinstance(item, dict)
                and item.get("status") == 1
                and item.get("name") != "report"
            ]
        return len(running_tasks) > 1

    @staticmethod
    def _now_iso() -> str:
        return datetime.now().astimezone().isoformat(timespec="milliseconds")

    def _set_state(self, state: FollowState) -> None:
        if state == self.state:
            return
        now_monotonic = time.monotonic()
        now_iso = self._now_iso()
        previous_duration_s = now_monotonic - self._state_entered_monotonic
        old_state = self.state
        self.previous_state = self.state
        self.state = state
        self._state_sequence += 1
        self._last_transition = {
            "from": old_state,
            "to": state,
            "at_iso": now_iso,
            "run_elapsed_s": round(
                now_monotonic - self._run_started_monotonic,
                3,
            ),
            "previous_state_duration_s": round(previous_duration_s, 3),
        }
        self._state_entered_monotonic = now_monotonic
        self._state_entered_wall_iso = now_iso

    def _snapshot(self, output: TrackingOutput | None = None) -> StateSnapshot:
        now_monotonic = time.monotonic()
        now_epoch = time.time()
        now_iso = self._now_iso()
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
        timing = {
            "timestamp_iso": now_iso,
            "timestamp_epoch_s": round(now_epoch, 3),
            "run_started_at_iso": self._run_started_wall_iso,
            "run_elapsed_s": round(
                now_monotonic - self._run_started_monotonic,
                3,
            ),
            "state_entered_at_iso": self._state_entered_wall_iso,
            "state_elapsed_s": round(
                now_monotonic - self._state_entered_monotonic,
                3,
            ),
            "state_sequence": self._state_sequence,
            "last_transition": self._last_transition,
        }
        return StateSnapshot(
            state=self.state,
            previous_state=self.previous_state,
            locked_track_id=self.locked_track_id,
            lock_seen_frames=self.lock_seen_frames,
            last_error=self.last_error,
            patrol_context=self.patrol_context,
            tracking=tracking,
            detection_debug=self.detection_debug,
            timing=timing,
        )

    def _select_lock_candidate(self, frame: FrameInput):
        candidates = []
        person_count = 0
        score_rejected_count = 0
        height_rejected_count = 0
        best_person = None
        best_person_score = None
        best_person_height_ratio = None
        max_person_height_ratio = None

        for item in frame.detections:
            if item.class_name != "person":
                continue

            person_count += 1
            bbox_height = item.bbox.y2 - item.bbox.y1
            height_ratio = bbox_height / max(frame.image_height, 1)

            if best_person is None or item.score > best_person.score:
                best_person = item
                best_person_score = item.score
                best_person_height_ratio = height_ratio
            if max_person_height_ratio is None or height_ratio > max_person_height_ratio:
                max_person_height_ratio = height_ratio

            if item.score < self.min_person_score:
                score_rejected_count += 1
                continue
            if height_ratio < self.min_person_height_ratio:
                height_rejected_count += 1
                continue

            candidates.append(item)

        if not frame.detections:
            reject_reason = "no_detections"
        elif person_count == 0:
            reject_reason = "no_person"
        elif candidates:
            reject_reason = "valid_person"
        elif score_rejected_count == person_count:
            reject_reason = "score_below_threshold"
        elif height_rejected_count == person_count:
            reject_reason = "height_ratio_below_threshold"
        else:
            reject_reason = "score_or_height_ratio_below_threshold"

        self.detection_debug = {
            "detections_count": len(frame.detections),
            "person_count": person_count,
            "valid_person_count": len(candidates),
            "best_person_score": best_person_score,
            "best_person_height_ratio": best_person_height_ratio,
            "max_person_height_ratio": max_person_height_ratio,
            "best_person_track_id": None if best_person is None else best_person.track_id,
            "min_person_score": self.min_person_score,
            "min_person_height_ratio": self.min_person_height_ratio,
            "score_rejected_count": score_rejected_count,
            "height_rejected_count": height_rejected_count,
            "reject_reason": reject_reason,
        }

        if not candidates:
            return None
        return max(candidates, key=lambda item: item.score)

    def _is_valid_person(self, detection, frame: FrameInput) -> bool:
        if detection.class_name != "person":
            return False
        if detection.score < self.min_person_score:
            return False
        bbox_height = detection.bbox.y2 - detection.bbox.y1
        height_ratio = bbox_height / max(frame.image_height, 1)
        return height_ratio >= self.min_person_height_ratio


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="巡检中断追人状态机")
    parser.add_argument("--yolo-ws", default=DEFAULT_YOLO_WS)
    parser.add_argument("--ip", default=DEFAULT_IP)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument(
        "--map-name",
        default=os.getenv("PATROL_MAP_NAME", "0317_1"),
        help="Map name used when rebuilding remaining patrol workflow.",
    )
    parser.add_argument(
        "--route-name",
        default=os.getenv("PATROL_ROUTE_NAME"),
        help="Route name fallback when robot context does not provide it.",
    )
    parser.add_argument(
        "--no-rebuild-on-bad-resume",
        action="store_true",
        help="Disable rebuilding remaining patrol workflow after bad resume state.",
    )
    parser.add_argument(
        "--yolo-recv-timeout-s",
        type=float,
        default=0.5,
        help="Seconds to wait for the next YOLO frame before injecting an empty frame while not in PATROL.",
    )
    parser.add_argument(
        "--max-empty-frames-after-timeout",
        type=int,
        default=60,
        help="Maximum consecutive synthetic empty frames after YOLO stops sending frames.",
    )
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


async def run_yolo_state_machine(args: argparse.Namespace) -> None:
    websockets = _websockets()
    token = _load_env_token()
    uri = normalize_yolo_ws_uri(args.yolo_ws, token) if token else args.yolo_ws
    robot = None if args.dry_run else Controller((args.ip, args.port))
    tracking = TrackingManager(robot=robot, dry_run=args.dry_run)
    machine = PersonFollowStateMachine(
        tracking=tracking,
        map_name=args.map_name,
        route_name=args.route_name,
        rebuild_on_bad_resume=not args.no_rebuild_on_bad_resume,
    )
    last_frame: FrameInput | None = None
    timeout_empty_frames = 0
    async with websockets.connect(uri, ping_interval=None) as ws:
        while True:
            try:
                raw = await asyncio.wait_for(
                    ws.recv(),
                    timeout=max(0.1, args.yolo_recv_timeout_s),
                )
            except asyncio.TimeoutError:
                if last_frame is None or machine.state == FollowState.PATROL:
                    continue
                if timeout_empty_frames >= max(0, args.max_empty_frames_after_timeout):
                    print(
                        json.dumps(
                            {
                                "event": "yolo_timeout_empty_frame_limit_reached",
                                "state": machine.state,
                                "empty_frames": timeout_empty_frames,
                                "timestamp_iso": PersonFollowStateMachine._now_iso(),
                            }
                        ),
                        file=sys.stderr,
                        flush=True,
                    )
                    continue

                timeout_empty_frames += 1
                frame = FrameInput(
                    timestamp=int(time.time() * 1000),
                    image_width=last_frame.image_width,
                    image_height=last_frame.image_height,
                    detections=[],
                )
                snapshot = await machine.handle_frame(frame)
                print(json.dumps(asdict(snapshot)), flush=True)
                continue
            except Exception as exc:
                print(
                    json.dumps(
                        {
                            "event": "yolo_websocket_receive_failed",
                            "error": str(exc),
                            "timestamp_iso": PersonFollowStateMachine._now_iso(),
                        }
                    ),
                    file=sys.stderr,
                    flush=True,
                )
                raise

            msg = json.loads(raw)
            if msg.get("type") == "ping":
                await ws.send(json.dumps({"type": "pong", "ts": msg.get("ts")}))
                continue
            frame = FrameInput.from_yolo_ws_payload(msg)
            last_frame = frame
            timeout_empty_frames = 0
            snapshot = await machine.handle_frame(frame)
            print(json.dumps(asdict(snapshot)), flush=True)


if __name__ == "__main__":
    asyncio.run(run_yolo_state_machine(parse_args()))
