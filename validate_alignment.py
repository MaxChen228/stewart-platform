#!/usr/bin/env python3

from __future__ import annotations

import json
import time
import urllib.request


BASE = "http://127.0.0.1:8080"


def api_get(path: str) -> dict:
    with urllib.request.urlopen(f"{BASE}{path}", timeout=5) as response:
        return json.load(response)


def api_post(path: str, payload: dict) -> dict:
    req = urllib.request.Request(
        f"{BASE}{path}",
        data=json.dumps(payload).encode(),
        headers={"Content-Type": "application/json"},
    )
    with urllib.request.urlopen(req, timeout=5) as response:
        return json.load(response)


def wait(seconds: float = 1.5) -> dict:
    time.sleep(seconds)
    return api_get("/api/state")


def motor_summary(state: dict) -> list[tuple[int, float, float]]:
    rows = []
    for idx, motor in enumerate(state["hardware"]["motors"], start=1):
        target = state["solution"]["servo_angles_deg"][idx - 1]
        actual = motor["deg"]
        rows.append((idx, target, actual))
    return rows


def print_state(title: str, state: dict) -> None:
    print(f"\n=== {title} ===")
    print(
        f"Pose z={state['pose']['z']:.2f}  CalZ={state['alignment']['calibrationZ']:.2f}  "
        f"Residual={state['actualSolution']['residualNorm']:.3f}  "
        f"Converged={state['actualSolution']['converged']}"
    )
    for idx, target, actual in motor_summary(state):
        print(f"M{idx}: target={target:8.2f} actual={actual:8.2f} error={actual - target:8.2f}")


def main() -> int:
    state = api_get("/api/state")
    cal_z = state["alignment"]["calibrationZ"]

    api_post("/api/mode", {"mode": "SIM+HW", "liveSend": False})
    api_post("/api/command", {"command": "enable_all"})

    down_pose = dict(state["pose"])
    down_pose["z"] = cal_z - 5.0
    api_post("/api/pose", {"pose": down_pose, "applyHardware": True})
    moved = wait()
    print_state("Moved -5mm from calibration", moved)

    home_pose = dict(state["pose"])
    home_pose["z"] = cal_z
    api_post("/api/pose", {"pose": home_pose, "applyHardware": True})
    returned = wait()
    print_state("Returned to calibration", returned)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
