from __future__ import annotations

import json
import sys
import time
import urllib.error
import urllib.request
from dataclasses import dataclass

import serial


PORT = "COM4"
BAUD = 115200
NODE_ID = "esp32_bedroom"
SERVER_URL = "http://127.0.0.1:8000/api/radar/esp32_bedroom"


@dataclass
class Target:
    valid: bool
    x_cm: float
    y_cm: float
    speed_cm_s: float
    gate_mm: int


def parse_plot_line(line: str) -> list[Target] | None:
    if not line.startswith("PLOT,"):
        return None

    parts = line.strip().split(",")
    if len(parts) != 1 + 5 * 3:
        return None

    targets: list[Target] = []
    for index in range(1, len(parts), 5):
        targets.append(
            Target(
                valid=parts[index] == "1",
                x_cm=float(parts[index + 1]),
                y_cm=float(parts[index + 2]),
                speed_cm_s=float(parts[index + 3]),
                gate_mm=int(parts[index + 4]),
            )
        )
    return targets


def build_payload(targets: list[Target]) -> bytes:
    data = {
        "node_id": NODE_ID,
        "ts_ms": int(time.time() * 1000),
        "node_pose": {"x_m": 0.0, "y_m": 0.0, "yaw_deg": 0.0},
        "targets": [],
    }

    for idx, target in enumerate(targets, start=1):
        if not target.valid:
            continue
        data["targets"].append(
            {
                "target_id": idx,
                "x_m": target.x_cm / 100.0,
                "y_m": target.y_cm / 100.0,
                "speed_m_s": target.speed_cm_s / 100.0,
                "gate_m": target.gate_mm / 1000.0,
            }
        )

    return json.dumps(data).encode("utf-8")


def post_payload(payload: bytes) -> None:
    request = urllib.request.Request(
        SERVER_URL,
        data=payload,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=2) as response:
        response.read()


def run(port: str) -> None:
    print(f"Serial bridge starting on {port} -> {SERVER_URL}")
    while True:
        try:
            with serial.Serial(port, BAUD, timeout=1) as ser:
                print(f"Connected to {port}")
                while True:
                    line = ser.readline().decode("utf-8", errors="ignore").strip()
                    targets = parse_plot_line(line)
                    if targets is None:
                        continue
                    try:
                        post_payload(build_payload(targets))
                    except urllib.error.URLError as exc:
                        print(f"POST failed: {exc}")
                        time.sleep(0.5)
        except serial.SerialException as exc:
            print(f"Serial error: {exc}")
            time.sleep(1.0)


if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else PORT
    run(port)
