from __future__ import annotations

import asyncio
import json
import os
import time
from contextlib import suppress
from dataclasses import asdict
from datetime import datetime
from enum import Enum
from typing import Any, Dict, Optional

from fastapi import FastAPI, HTTPException
from pydantic import BaseModel

from person_follow_state_machine import (
    FollowState,
    PersonFollowStateMachine,
    _load_env_token,
    _websockets,
)
from p4_tracker.controller import FrameInput
from totalController import Controller
from tracking_manager import TrackingManager
from track_live import DEFAULT_IP, DEFAULT_PORT, normalize_yolo_ws_uri


app = FastAPI(title="Person Follow State Machine API", version="0.1.0")


class PersonFollowStartRequest(BaseModel):
    yolo_ws: str
    robot_ip: str = DEFAULT_IP
    robot_port: int = DEFAULT_PORT
    map_name: str = os.getenv("PATROL_MAP_NAME", "0317_1")
    route_name: Optional[str] = os.getenv("PATROL_ROUTE_NAME")
    dry_run: bool = False
    lock_confirm_frames: int = 5
    min_person_score: float = 0.70
    min_person_height_ratio: float = 0.15
    pause_reason: str = "person_follow_interrupt"
    rebuild_on_bad_resume: bool = True
    yolo_recv_timeout_s: float = 0.5
    max_empty_frames_after_timeout: int = 60


class PersonFollowStopRequest(BaseModel):
    reason: str = "manual_stop"


PERSON_FOLLOW_RUNTIME_STATE: Dict[str, Any] = {
    "running": False,
    "params": None,
    "last_snapshot": None,
    "last_error": None,
    "last_event": None,
    "started_at": None,
    "stopped_at": None,
}

_state_lock = asyncio.Lock()
_follow_task: asyncio.Task | None = None
_follow_machine: PersonFollowStateMachine | None = None


def _now_iso() -> str:
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def _snapshot_to_dict(snapshot) -> dict[str, Any]:
    data = asdict(snapshot)
    data["state"] = snapshot.state.value
    data["previous_state"] = snapshot.previous_state.value
    return _json_safe(data)


def _json_safe(value: Any) -> Any:
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_json_safe(item) for item in value]
    if isinstance(value, tuple):
        return [_json_safe(item) for item in value]
    return value


def _request_to_dict(req: PersonFollowStartRequest) -> dict[str, Any]:
    if hasattr(req, "model_dump"):
        return req.model_dump()
    return req.dict()


async def _record_state(**updates: Any) -> None:
    async with _state_lock:
        PERSON_FOLLOW_RUNTIME_STATE.update(updates)


async def _run_person_follow(req: PersonFollowStartRequest) -> None:
    global _follow_machine

    websockets = _websockets()
    token = _load_env_token()
    uri = normalize_yolo_ws_uri(req.yolo_ws, token) if token else req.yolo_ws
    robot = None if req.dry_run else Controller((req.robot_ip, req.robot_port))
    tracking = TrackingManager(robot=robot, dry_run=req.dry_run)
    machine = PersonFollowStateMachine(
        tracking=tracking,
        lock_confirm_frames=req.lock_confirm_frames,
        min_person_score=req.min_person_score,
        min_person_height_ratio=req.min_person_height_ratio,
        pause_reason=req.pause_reason,
        map_name=req.map_name,
        route_name=req.route_name,
        rebuild_on_bad_resume=req.rebuild_on_bad_resume,
    )
    _follow_machine = machine
    last_frame: FrameInput | None = None
    timeout_empty_frames = 0

    try:
        async with websockets.connect(uri, ping_interval=None) as ws:
            await _record_state(last_event="yolo_websocket_connected")
            while True:
                try:
                    raw = await asyncio.wait_for(
                        ws.recv(),
                        timeout=max(0.1, req.yolo_recv_timeout_s),
                    )
                except asyncio.TimeoutError:
                    if last_frame is None or machine.state == FollowState.PATROL:
                        continue
                    if timeout_empty_frames >= max(
                        0,
                        req.max_empty_frames_after_timeout,
                    ):
                        await _record_state(
                            last_event={
                                "event": "yolo_timeout_empty_frame_limit_reached",
                                "state": machine.state.value,
                                "empty_frames": timeout_empty_frames,
                                "timestamp_iso": _now_iso(),
                            }
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
                    await _record_state(last_snapshot=_snapshot_to_dict(snapshot))
                    continue

                msg = json.loads(raw)
                if msg.get("type") == "ping":
                    await ws.send(json.dumps({"type": "pong", "ts": msg.get("ts")}))
                    continue

                frame = FrameInput.from_yolo_ws_payload(msg)
                last_frame = frame
                timeout_empty_frames = 0
                snapshot = await machine.handle_frame(frame)
                await _record_state(
                    last_snapshot=_snapshot_to_dict(snapshot),
                    last_error=snapshot.last_error,
                )
    except asyncio.CancelledError:
        raise
    except Exception as exc:
        await _record_state(last_error=str(exc), last_event="person_follow_failed")
    finally:
        if _follow_machine is machine:
            with suppress(Exception):
                await machine.stop(reason="api_task_finished")
            _follow_machine = None
        await _record_state(running=False, stopped_at=_now_iso())


def _is_task_running() -> bool:
    return _follow_task is not None and not _follow_task.done()


@app.post("/person-follow/start")
async def person_follow_start(req: PersonFollowStartRequest):
    global _follow_task

    async with _state_lock:
        if _is_task_running():
            raise HTTPException(status_code=409, detail="person follow is already running")

        params = _request_to_dict(req)
        PERSON_FOLLOW_RUNTIME_STATE.update(
            {
                "running": True,
                "params": params,
                "last_snapshot": None,
                "last_error": None,
                "last_event": "starting",
                "started_at": _now_iso(),
                "stopped_at": None,
            }
        )
        _follow_task = asyncio.create_task(_run_person_follow(req))

    return {
        "ok": True,
        "message": "person follow started",
        "state": PERSON_FOLLOW_RUNTIME_STATE,
    }


@app.post("/person-follow/stop")
async def person_follow_stop(req: PersonFollowStopRequest):
    global _follow_task

    task = _follow_task
    machine = _follow_machine
    if task is None or task.done():
        await _record_state(running=False, stopped_at=_now_iso())
        return {
            "ok": True,
            "message": "person follow is not running",
            "state": PERSON_FOLLOW_RUNTIME_STATE,
        }

    if machine is not None:
        with suppress(Exception):
            snapshot = await machine.stop(reason=req.reason)
            await _record_state(last_snapshot=_snapshot_to_dict(snapshot))

    task.cancel()
    with suppress(asyncio.CancelledError):
        await task
    _follow_task = None
    await _record_state(
        running=False,
        last_event={"event": "stopped", "reason": req.reason, "timestamp_iso": _now_iso()},
        stopped_at=_now_iso(),
    )
    return {
        "ok": True,
        "message": "person follow stopped",
        "state": PERSON_FOLLOW_RUNTIME_STATE,
    }


@app.get("/person-follow/status")
async def person_follow_status():
    task_running = _is_task_running()
    async with _state_lock:
        PERSON_FOLLOW_RUNTIME_STATE["running"] = task_running
        state = dict(PERSON_FOLLOW_RUNTIME_STATE)

    return {"ok": True, "state": state}
