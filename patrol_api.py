import asyncio
import json
import uuid
from typing import Any, Dict, List, Optional
from xml.etree.ElementTree import Element, SubElement, tostring

import websockets
from fastapi import FastAPI, HTTPException, Query
from pydantic import BaseModel


# WS_URL = "ws://10.65.233.62:9092"
WS_URL = "ws://10.69.235.139:9092"
DEFAULT_TIMEOUT = 15


PATROL_RUNTIME_STATE: Dict[str, Any] = {
    "mode": "idle",  # idle / prepared / ready_to_start / running / stopping / error
    "map_name": None,
    "route_name": None,
    "workflow_name": None,
    "last_error": None,
    "last_response": None,
    "path_data": None,
    "localization_mode": "external_software",
}


class PatrolPrepareRequest(BaseModel):
    map_name: str
    stand_up_first: bool = True
    switch_auto_mode: bool = True


class PatrolStartRequest(BaseModel):
    map_name: str
    route_name: str


class PatrolStopRequest(BaseModel):
    go_lie_down: bool = False


class WorkflowPauseRequest(BaseModel):
    reason: str = "Python测试"


app = FastAPI(title="Patrol Context Adapter API", version="0.3.0")


def make_msg(msg_type: str, task: str, params: Optional[dict] = None, **extra) -> dict:
    msg = {
        "id": str(uuid.uuid4()),
        "type": msg_type,
        "task": task,
    }
    if params is not None:
        msg["params"] = params
    msg.update(extra)
    return msg


async def send_and_wait(ws, payload: dict, timeout: float = DEFAULT_TIMEOUT) -> dict:
    await ws.send(json.dumps(payload, ensure_ascii=False))
    while True:
        raw = await asyncio.wait_for(ws.recv(), timeout=timeout)
        data = json.loads(raw)
        if data.get("id") == payload["id"]:
            return data


async def ensure_success(ws, payload: dict, timeout: float = DEFAULT_TIMEOUT) -> dict:
    resp = await send_and_wait(ws, payload, timeout=timeout)
    if resp.get("success") != 1:
        raise RuntimeError(resp.get("message", f"request failed: {resp}"))
    return resp


def as_text(value: Any, default: str = "0") -> str:
    if value is None:
        return default
    return str(value)


def resolve_gait_cmd_code(obstacle_gait: Any) -> str:
    if obstacle_gait == "grip":
        return str(0x21010402)
    if obstacle_gait == "general":
        return str(0x21010401)
    if obstacle_gait == "high_step":
        return str(0x21010407)
    return str(0x21010300)


def build_behavior_tree_xml(route_name: str, points: List[Dict[str, Any]]) -> str:
    root = Element("root", {"BTCPP_format": "4"})
    behavior_tree = SubElement(root, "BehaviorTree")
    main_seq = SubElement(behavior_tree, "Sequence")

    SubElement(main_seq, "InspectionDataCollector", {"action": "clear"})

    for i, point in enumerate(points, start=1):
        point_id = f"point_{i}"
        point_name = as_text(point.get("point_name"), f"导航点{i}")
        pos = point.get("position") or {}
        ori = point.get("orientation") or {}

        task_updater = SubElement(main_seq, "TaskStateUpdater", {"task_id": point_id})
        nav_seq = SubElement(task_updater, "Sequence")

        SubElement(
            nav_seq,
            "SetNavigationDestination",
            {
                "name": f"导航到巡检点{i}",
                "posx": as_text(pos.get("x")),
                "posy": as_text(pos.get("y")),
                "posz": as_text(pos.get("z")),
                "orix": as_text(ori.get("x")),
                "oriy": as_text(ori.get("y")),
                "oriz": as_text(ori.get("z")),
                "oriw": as_text(ori.get("w"), "1"),
            },
        )
        SubElement(nav_seq, "Sleep", {"name": f"等待导航点到位{i}", "msec": "3000"})

        operation = point.get("operation") or {}
        obstacle_enabled = bool(operation.get("obstacle_enabled"))
        obstacle_gait = operation.get("obstacle_gait")
        cmd_code = resolve_gait_cmd_code(obstacle_gait) if obstacle_enabled else str(0x21010300)

        SubElement(
            nav_seq,
            "RobotDogSimpleCmd",
            {
                "cmd_code": cmd_code,
                "cmd_value": "0",
                "cmd_type": "0",
                "wait_result": "false",
                "timeout": "1.0",
            },
        )

        SubElement(
            main_seq,
            "InspectionDataCollector",
            {
                "name": f"收集导航点{i}数据",
                "action": "collect",
                "task_id": point_id,
                "waypoint_name": point_name,
                "position_x": as_text(pos.get("x")),
                "position_y": as_text(pos.get("y")),
                "position_z": as_text(pos.get("z")),
                "orientation_x": as_text(ori.get("x")),
                "orientation_y": as_text(ori.get("y")),
                "orientation_z": as_text(ori.get("z")),
                "orientation_w": as_text(ori.get("w"), "1"),
            },
        )

    SubElement(
        main_seq,
        "InspectionDataCollector",
        {"name": "获取巡检数据", "action": "get", "waypoints": "{waypoints}"},
    )
    report_task = SubElement(main_seq, "TaskStateUpdater", {"task_id": "report"})
    SubElement(
        report_task,
        "GenerateInspectionReport",
        {"name": "生成巡检报告", "route_name": route_name, "waypoints": "{waypoints}"},
    )

    xml_body = tostring(root, encoding="unicode")
    return '<?xml version="1.0" encoding="UTF-8"?>' + xml_body


async def stand_up(ws) -> dict:
    payload = make_msg(
        "robot_dog_control",
        "simple_cmd",
        cmd_code=553714178,
        cmd_value=0,
        cmd_type=0,
    )
    return await ensure_success(ws, payload)


async def lie_down(ws) -> dict:
    payload = make_msg(
        "robot_dog_control",
        "simple_cmd",
        cmd_code=553714178,
        cmd_value=0,
        cmd_type=0,
    )
    return await ensure_success(ws, payload)


async def switch_to_auto_mode(ws) -> dict:
    payload = make_msg(
        "robot_dog_control",
        "simple_cmd",
        cmd_code=553716739,
        cmd_value=0,
        cmd_type=0,
    )
    return await ensure_success(ws, payload)


async def start_navigation(ws, map_name: str) -> dict:
    payload = make_msg(
        "navigation",
        "set_navigation_status",
        {"flag": True, "map_name": map_name},
    )
    return await ensure_success(ws, payload)


async def stop_navigation(ws) -> dict:
    payload = make_msg(
        "navigation",
        "set_navigation_status",
        {"flag": False, "map_name": ""},
    )
    return await ensure_success(ws, payload)


async def get_maps(ws) -> dict:
    payload = make_msg("navigation", "get_maps")
    return await ensure_success(ws, payload)


async def get_patrol_routes(ws, map_name: str) -> dict:
    payload = make_msg(
        "inspection_robot_navigation",
        "get_paths",
        {"map_name": map_name},
    )
    return await ensure_success(ws, payload)


async def get_patrol_path(ws, map_name: str, route_name: str) -> dict:
    payload = make_msg(
        "inspection_robot_navigation",
        "get_path",
        {"map_name": map_name, "path_name": route_name},
    )
    return await ensure_success(ws, payload)


async def upload_behavior_tree(ws, workflow_name: str, xml_content: str) -> dict:
    payload = make_msg(
        "behaviortree",
        "set_tree_content",
        {"workflow_name": workflow_name, "content": xml_content},
    )
    return await ensure_success(ws, payload)


async def start_patrol_workflow(ws, workflow_name: str) -> dict:
    payload = make_msg(
        "robot",
        "start_workflow",
        {"workflow_name": workflow_name},
    )
    return await ensure_success(ws, payload)


async def stop_patrol_workflow(ws) -> dict:
    payload = make_msg("robot", "stop_workflow")
    return await ensure_success(ws, payload)


async def get_robot_status(ws) -> dict:
    payload = make_msg("robot", "get_status")
    return await send_and_wait(ws, payload)


async def get_robot_context(ws) -> dict:
    payload = make_msg("robot", "get_context")
    return await send_and_wait(ws, payload)


async def pause_patrol_workflow(ws, reason: str) -> dict:
    payload = make_msg("robot", "pause_workflow", {"reason": reason})
    return await send_and_wait(ws, payload)


async def resume_patrol_workflow(ws) -> dict:
    payload = make_msg("robot", "resume_workflow")
    return await send_and_wait(ws, payload)


async def do_prepare(map_name: str, stand_up_first: bool, switch_auto: bool):
    async with websockets.connect(WS_URL) as ws:
        step_responses = {}

        if stand_up_first:
            step_responses["stand_up"] = await stand_up(ws)
            await asyncio.sleep(1)

        if switch_auto:
            step_responses["auto_mode"] = await switch_to_auto_mode(ws)
            await asyncio.sleep(1)

        step_responses["navigation"] = await start_navigation(ws, map_name)
        await asyncio.sleep(1)

        PATROL_RUNTIME_STATE.update({
            "mode": "prepared",
            "map_name": map_name,
            "route_name": None,
            "workflow_name": None,
            "path_data": None,
            "last_error": None,
            "last_response": step_responses,
        })


async def do_start(map_name: str, route_name: str):
    async with websockets.connect(WS_URL) as ws:
        path_resp = await get_patrol_path(ws, map_name, route_name)
        path_data = path_resp.get("data") or {}
        points = path_data.get("points") or []
        if not points:
            raise RuntimeError("路线 points 为空，无法启动巡检")

        xml_content = build_behavior_tree_xml(route_name, points)
        tree_resp = await upload_behavior_tree(ws, route_name, xml_content)
        await asyncio.sleep(0.5)

        start_resp = await start_patrol_workflow(ws, route_name)

        PATROL_RUNTIME_STATE.update({
            "mode": "running",
            "map_name": map_name,
            "route_name": route_name,
            "workflow_name": route_name,
            "path_data": path_data,
            "last_error": None,
            "last_response": {
                "path": path_resp,
                "set_tree": tree_resp,
                "start_workflow": start_resp,
            },
        })


async def do_stop(go_lie_down: bool):
    async with websockets.connect(WS_URL) as ws:
        stop_resp = await stop_patrol_workflow(ws)
        await asyncio.sleep(0.5)

        nav_resp = await stop_navigation(ws)
        await asyncio.sleep(0.5)

        lie_resp = None
        if go_lie_down:
            lie_resp = await lie_down(ws)

        PATROL_RUNTIME_STATE.update({
            "mode": "idle",
            "last_error": None,
            "last_response": {
                "stop_workflow": stop_resp,
                "stop_navigation": nav_resp,
                "lie_down": lie_resp,
            },
        })


@app.get("/maps")
async def api_get_maps():
    try:
        async with websockets.connect(WS_URL) as ws:
            resp = await get_maps(ws)
        maps = resp.get("data", {}).get("maps", [])
        return {"ok": True, "maps": maps, "count": len(maps)}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/patrol/routes")
async def api_get_patrol_routes(map_name: str = Query(...)):
    try:
        async with websockets.connect(WS_URL) as ws:
            resp = await get_patrol_routes(ws, map_name)

        routes = resp.get("data", []) or []
        simplified = [
            {
                "path_name": item.get("path_name"),
                "map_name": item.get("map_name"),
                "category": item.get("category"),
                "points_count": len(item.get("points") or []),
            }
            for item in routes
        ]
        return {"ok": True, "map_name": map_name, "routes": simplified, "count": len(simplified)}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/patrol/route-detail")
async def api_get_patrol_route_detail(
    map_name: str = Query(...),
    route_name: str = Query(...),
):
    try:
        async with websockets.connect(WS_URL) as ws:
            resp = await get_patrol_path(ws, map_name, route_name)

        path_data = resp.get("data") or {}
        points = path_data.get("points") or []
        return {
            "ok": True,
            "map_name": map_name,
            "route_name": route_name,
            "points_count": len(points),
            "path_data": path_data,
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/patrol/prepare")
async def patrol_prepare(req: PatrolPrepareRequest):
    try:
        await do_prepare(
            map_name=req.map_name,
            stand_up_first=req.stand_up_first,
            switch_auto=req.switch_auto_mode,
        )
        return {
            "ok": True,
            "message": "prepare success, please do relocalization by software before /patrol/start",
            "state": PATROL_RUNTIME_STATE,
        }
    except Exception as e:
        PATROL_RUNTIME_STATE["mode"] = "error"
        PATROL_RUNTIME_STATE["last_error"] = str(e)
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/patrol/start")
async def patrol_start(req: PatrolStartRequest):
    try:
        if PATROL_RUNTIME_STATE.get("map_name") != req.map_name:
            raise RuntimeError("map_name 与当前 prepare 阶段不一致，请先重新 prepare")

        await do_start(
            map_name=req.map_name,
            route_name=req.route_name,
        )
        return {
            "ok": True,
            "message": "patrol started",
            "state": PATROL_RUNTIME_STATE,
        }
    except Exception as e:
        PATROL_RUNTIME_STATE["mode"] = "error"
        PATROL_RUNTIME_STATE["last_error"] = str(e)
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/patrol/stop")
async def patrol_stop(req: PatrolStopRequest):
    try:
        PATROL_RUNTIME_STATE["mode"] = "stopping"
        await do_stop(go_lie_down=req.go_lie_down)
        return {
            "ok": True,
            "message": "patrol stopped",
            "state": PATROL_RUNTIME_STATE,
        }
    except Exception as e:
        PATROL_RUNTIME_STATE["mode"] = "error"
        PATROL_RUNTIME_STATE["last_error"] = str(e)
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/patrol/lie-down")
async def patrol_lie_down():
    try:
        async with websockets.connect(WS_URL) as ws:
            resp = await lie_down(ws)
        return {"ok": True, "message": "lie down success", "response": resp}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/patrol/status")
async def patrol_status():
    return {
        "ok": True,
        "state": {
            "mode": PATROL_RUNTIME_STATE["mode"],
            "map_name": PATROL_RUNTIME_STATE["map_name"],
            "route_name": PATROL_RUNTIME_STATE["route_name"],
            "workflow_name": PATROL_RUNTIME_STATE["workflow_name"],
            "last_error": PATROL_RUNTIME_STATE["last_error"],
            "localization_mode": PATROL_RUNTIME_STATE["localization_mode"],
        }
    }


@app.get("/patrol/context")
async def patrol_context():
    path_data = PATROL_RUNTIME_STATE.get("path_data") or {}
    return {
        "ok": True,
        "context": {
            "map_name": PATROL_RUNTIME_STATE["map_name"],
            "route_name": PATROL_RUNTIME_STATE["route_name"],
            "workflow_name": PATROL_RUNTIME_STATE["workflow_name"],
            "localization_mode": PATROL_RUNTIME_STATE["localization_mode"],
            "path_data": path_data,
            "points_count": len(path_data.get("points") or []),
        }
    }


@app.get("/robot/status")
async def robot_status():
    try:
        async with websockets.connect(WS_URL) as ws:
            resp = await get_robot_status(ws)
        return {"ok": True, "response": resp}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/robot/context")
async def robot_context():
    try:
        async with websockets.connect(WS_URL) as ws:
            resp = await get_robot_context(ws)
        return {"ok": True, "response": resp}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/robot/pause")
async def robot_pause(req: WorkflowPauseRequest):
    try:
        async with websockets.connect(WS_URL) as ws:
            resp = await pause_patrol_workflow(ws, req.reason)
        PATROL_RUNTIME_STATE["last_response"] = {"pause_workflow": resp}
        return {"ok": True, "message": "workflow paused", "response": resp}
    except Exception as e:
        PATROL_RUNTIME_STATE["last_error"] = str(e)
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/robot/resume")
async def robot_resume():
    try:
        async with websockets.connect(WS_URL) as ws:
            resp = await resume_patrol_workflow(ws)
        PATROL_RUNTIME_STATE["last_response"] = {"resume_workflow": resp}
        return {"ok": True, "message": "workflow resumed", "response": resp}
    except Exception as e:
        PATROL_RUNTIME_STATE["last_error"] = str(e)
        raise HTTPException(status_code=500, detail=str(e))

# commit test