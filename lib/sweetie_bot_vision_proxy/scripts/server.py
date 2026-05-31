#!/usr/bin/env python3

import asyncio
import json
import os
import struct
import time
from typing import Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.responses import PlainTextResponse
import uvicorn


app = FastAPI()

# Header:
# magic[4], version u16, header_size u16,
# frame_id u64, capture_ts_ns u64,
# width u32, height u32, jpeg_size u32, flags u32
#
# Little-endian, packed.
FRAME_MAGIC = b"MJPG"
FRAME_HEADER_STRUCT = struct.Struct("<4sHHQQIIIiI")
FRAME_VERSION = 1

API_KEY = os.getenv("VISION_API_KEY", "")
PROCESSING_DELAY_SEC = float(os.getenv("VISION_PROCESSING_DELAY_SEC", "0.05"))


def now_ns() -> int:
    return time.time_ns()


def parse_frame_message(data: bytes) -> dict:
    if len(data) < FRAME_HEADER_STRUCT.size:
        raise ValueError(f"message too short: {len(data)} bytes")
    (
        magic,
        version,
        header_size,
        frame_id,
        capture_ts_ns,
        width,
        height,
        jpeg_size,
        camera_rotation_deg,
        flags,
    ) = FRAME_HEADER_STRUCT.unpack_from(data, 0)

    if magic != FRAME_MAGIC:
        raise ValueError(f"bad magic: {magic!r}")

    if version != FRAME_VERSION:
        raise ValueError(f"bad version: {version}")

    if header_size != FRAME_HEADER_STRUCT.size:
        raise ValueError(f"bad header_size: {header_size}")

    expected_size = header_size + jpeg_size
    if len(data) != expected_size:
        raise ValueError(
            f"bad jpeg_size: header says {jpeg_size}, "
            f"message has {len(data) - header_size}"
        )

    jpeg = data[header_size:]

    return {
        "frame_id": frame_id,
        "capture_ts_ns": capture_ts_ns,
        "width": width,
        "height": height,
        "jpeg_size": jpeg_size,
        "camera_rotation_deg": camera_rotation_deg,
        "flags": flags,
        "jpeg": jpeg,
    }


def check_auth(websocket: WebSocket) -> None:
    if not API_KEY:
        return

    auth = websocket.headers.get("authorization", "")
    expected = f"Bearer {API_KEY}"

    if auth != expected:
        raise HTTPException(status_code=401, detail="Unauthorized")


@app.get("/health")
async def health():
    return PlainTextResponse("ok\n")


@app.websocket("/ws")
async def ws_endpoint(websocket: WebSocket):
    check_auth(websocket)
    await websocket.accept()

    print("client connected")

    try:
        while True:
            data = await websocket.receive_bytes()

            server_recv_ts_ns = now_ns()

            try:
                frame = parse_frame_message(data)
            except Exception as e:
                await websocket.send_text(
                    json.dumps(
                        {
                            "type": "error",
                            "error": str(e),
                        }
                    )
                )
                continue

            # Здесь позже будет:
            #   JPEG decode
            #   inference
            #   postprocess
            #
            # Пока искусственная задержка.
            if PROCESSING_DELAY_SEC > 0:
                await asyncio.sleep(PROCESSING_DELAY_SEC)

            result = {
                "type": "result",
                "frame_id": frame["frame_id"],
                "capture_ts_ns": frame["capture_ts_ns"],
                "server_recv_ts_ns": server_recv_ts_ns,
                "server_send_ts_ns": now_ns(),
                "camera_rotation_deg": frame["camera_rotation_deg"],
                "image": {
                    "width": frame["width"],
                    "height": frame["height"],
                    "jpeg_size": frame["jpeg_size"],
                },
                "objects": [
                    {
                        "class": "test_object",
                        "confidence": 0.99,
                        "bbox": [0.25, 0.25, 0.50, 0.50],
                    }
                ],
            }

            await websocket.send_text(json.dumps(result, separators=(",", ":")))

    except WebSocketDisconnect:
        print("client disconnected")
    except Exception as e:
        print(f"connection error: {e!r}")


if __name__ == "__main__":
    uvicorn.run(
        "server:app",
        host="0.0.0.0",
        port=8080,
        reload=False,
        ws_ping_interval=20,
        ws_ping_timeout=20,
    )
