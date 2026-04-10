from __future__ import annotations

import asyncio
import json
import time
from contextlib import asynccontextmanager, suppress
from pathlib import Path
from typing import Any

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field


ROOT_DIR = Path(__file__).resolve().parent.parent
WEB_DIR = ROOT_DIR / "web"
CONFIG_DIR = ROOT_DIR / "config"
LAYOUT_PATH = CONFIG_DIR / "floorplans.json"

ROOM_WIDTH_M = 8.0
ROOM_HEIGHT_M = 6.0


class TargetPayload(BaseModel):
    target_id: int = Field(..., ge=1)
    x_m: float
    y_m: float
    speed_m_s: float = 0.0
    gate_m: float = 0.0


class RadarPayload(BaseModel):
    node_id: str
    ts_ms: int | None = None
    node_pose: dict[str, float] | None = None
    targets: list[TargetPayload] = Field(default_factory=list)


class LidarPointPayload(BaseModel):
    angle_deg: float
    distance_m: float
    intensity: int = Field(..., ge=0, le=255)
    x_m: float
    y_m: float


class LidarScanPayload(BaseModel):
    node_id: str
    source: str = "ld19"
    ts_ms: int | None = None
    speed_hz: float = 0.0
    scan_rate_hz: float = 0.0
    point_count: int = 0
    valid_count: int = 0
    max_distance_m: float = 12.0
    points: list[LidarPointPayload] = Field(default_factory=list)


class LayoutPayload(BaseModel):
    layout: dict[str, Any]
    source_client_id: str | None = None


class ConnectionManager:
    def __init__(self) -> None:
        self.connections: set[WebSocket] = set()

    async def connect(self, websocket: WebSocket) -> None:
        await websocket.accept()
        self.connections.add(websocket)

    def disconnect(self, websocket: WebSocket) -> None:
        self.connections.discard(websocket)

    async def broadcast_json(self, payload: dict[str, Any]) -> None:
        dead_connections: list[WebSocket] = []
        for websocket in self.connections:
            try:
                await websocket.send_json(payload)
            except Exception:
                dead_connections.append(websocket)

        for websocket in dead_connections:
            self.disconnect(websocket)


manager = ConnectionManager()

state: dict[str, Any] = {
    "meta": {
        "room": {"width_m": ROOM_WIDTH_M, "height_m": ROOM_HEIGHT_M},
        "mode": "live",
        "coordinate_space": "radar_local",
    },
    "nodes": {},
    "last_update_ms": 0,
}


def load_layout() -> dict[str, Any]:
    return json.loads(LAYOUT_PATH.read_text(encoding="utf-8"))


def save_layout(layout: dict[str, Any]) -> None:
    LAYOUT_PATH.write_text(
        json.dumps(layout, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


layout_config: dict[str, Any] = load_layout()
lidar_scans: dict[str, Any] = {}


def snapshot_state() -> dict[str, Any]:
    return json.loads(json.dumps(state))


def snapshot_lidar() -> dict[str, Any]:
    return json.loads(json.dumps(lidar_scans))


async def publish_state() -> None:
    state["last_update_ms"] = int(time.time() * 1000)
    await manager.broadcast_json({"type": "state", "payload": snapshot_state()})


async def publish_layout(source_client_id: str | None = None) -> None:
    await manager.broadcast_json(
        {
            "type": "layout",
            "payload": json.loads(json.dumps(layout_config)),
            "source_client_id": source_client_id,
        }
    )


async def publish_lidar_state() -> None:
    await manager.broadcast_json({"type": "lidar_state", "payload": snapshot_lidar()})


async def publish_lidar_scan(node_id: str) -> None:
    scan = lidar_scans.get(node_id)
    if not scan:
        return
    state["last_update_ms"] = int(time.time() * 1000)
    await manager.broadcast_json({"type": "lidar_scan", "payload": json.loads(json.dumps(scan))})


async def upsert_node(payload: RadarPayload) -> None:
    received_at_ms = int(time.time() * 1000)
    ts_ms = payload.ts_ms or received_at_ms
    node_pose = payload.node_pose or {"x_m": 1.4, "y_m": 0.4, "yaw_deg": 0.0}
    previous = state["nodes"].get(payload.node_id, {})
    packet_count = int(previous.get("packet_count", 0)) + 1
    previous_received_at_ms = previous.get("received_at_ms")
    previous_ema_interval_ms = previous.get("ema_interval_ms")
    ema_interval_ms = previous_ema_interval_ms

    if previous_received_at_ms:
        interval_ms = max(1, received_at_ms - int(previous_received_at_ms))
        if previous_ema_interval_ms is None:
            ema_interval_ms = float(interval_ms)
        else:
            ema_interval_ms = (float(previous_ema_interval_ms) * 0.65) + (
                float(interval_ms) * 0.35
            )

    update_hz = 0.0 if not ema_interval_ms else 1000.0 / float(ema_interval_ms)
    state["nodes"][payload.node_id] = {
        "node_id": payload.node_id,
        "ts_ms": ts_ms,
        "received_at_ms": received_at_ms,
        "node_pose": node_pose,
        "targets": [target.model_dump() for target in payload.targets],
        "target_count": len(payload.targets),
        "packet_count": packet_count,
        "ema_interval_ms": ema_interval_ms,
        "update_hz": round(update_hz, 2),
    }
    await publish_state()


async def upsert_lidar_scan(payload: LidarScanPayload) -> None:
    received_at_ms = int(time.time() * 1000)
    ts_ms = payload.ts_ms or received_at_ms
    previous = lidar_scans.get(payload.node_id, {})
    packet_count = int(previous.get("packet_count", 0)) + 1
    previous_received_at_ms = previous.get("received_at_ms")
    previous_ema_interval_ms = previous.get("ema_interval_ms")
    ema_interval_ms = previous_ema_interval_ms

    if previous_received_at_ms:
        interval_ms = max(1, received_at_ms - int(previous_received_at_ms))
        if previous_ema_interval_ms is None:
            ema_interval_ms = float(interval_ms)
        else:
            ema_interval_ms = (float(previous_ema_interval_ms) * 0.65) + (
                float(interval_ms) * 0.35
            )

    update_hz = 0.0 if not ema_interval_ms else 1000.0 / float(ema_interval_ms)
    lidar_scans[payload.node_id] = {
        "node_id": payload.node_id,
        "source": payload.source,
        "ts_ms": ts_ms,
        "received_at_ms": received_at_ms,
        "speed_hz": round(float(payload.speed_hz), 2),
        "scan_rate_hz": round(float(payload.scan_rate_hz), 2),
        "point_count": int(payload.point_count or len(payload.points)),
        "valid_count": int(payload.valid_count or len(payload.points)),
        "max_distance_m": round(float(payload.max_distance_m), 2),
        "packet_count": packet_count,
        "ema_interval_ms": ema_interval_ms,
        "update_hz": round(update_hz, 2),
        "points": [point.model_dump() for point in payload.points],
    }
    await publish_lidar_scan(payload.node_id)


@asynccontextmanager
async def lifespan(_: FastAPI):
    try:
        yield
    finally:
        with suppress(asyncio.CancelledError):
            await asyncio.sleep(0)


app = FastAPI(title="LD2450 Demo Server", lifespan=lifespan)
app.mount("/static", StaticFiles(directory=WEB_DIR), name="static")
app.mount("/config", StaticFiles(directory=CONFIG_DIR), name="config")


@app.get("/")
async def index() -> FileResponse:
    return FileResponse(WEB_DIR / "index.html")


@app.get("/api/state")
async def get_state() -> dict[str, Any]:
    return snapshot_state()


@app.get("/api/layout")
async def get_layout() -> dict[str, Any]:
    return json.loads(json.dumps(layout_config))


@app.get("/api/lidar")
async def get_lidar() -> dict[str, Any]:
    return snapshot_lidar()


@app.put("/api/layout")
async def put_layout(payload: LayoutPayload) -> dict[str, Any]:
    global layout_config
    layout_config = payload.layout
    save_layout(layout_config)
    await publish_layout(payload.source_client_id)
    return {"ok": True}


@app.post("/api/radar/{node_id}")
async def ingest_radar(node_id: str, payload: RadarPayload) -> dict[str, Any]:
    normalized_payload = payload.model_copy(update={"node_id": node_id})
    await upsert_node(normalized_payload)
    return {"ok": True, "node_id": node_id, "target_count": len(payload.targets)}


@app.post("/api/lidar/{node_id}")
async def ingest_lidar(node_id: str, payload: LidarScanPayload) -> dict[str, Any]:
    normalized_payload = payload.model_copy(update={"node_id": node_id})
    await upsert_lidar_scan(normalized_payload)
    return {"ok": True, "node_id": node_id, "point_count": len(payload.points)}


@app.delete("/api/lidar/{node_id}")
async def delete_lidar(node_id: str) -> dict[str, Any]:
    existed = node_id in lidar_scans
    lidar_scans.pop(node_id, None)
    if existed:
      await publish_lidar_state()
    return {"ok": True, "node_id": node_id, "deleted": existed}


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket) -> None:
    await manager.connect(websocket)
    await websocket.send_json({"type": "state", "payload": snapshot_state()})
    await websocket.send_json(
        {"type": "layout", "payload": json.loads(json.dumps(layout_config))}
    )
    await websocket.send_json({"type": "lidar_state", "payload": snapshot_lidar()})
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        manager.disconnect(websocket)
