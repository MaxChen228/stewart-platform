#!/usr/bin/env python3

from __future__ import annotations

import json
import sys
import urllib.error
import urllib.request


STATE_URL = "http://127.0.0.1:8080/api/state"


def fetch_state() -> dict:
    with urllib.request.urlopen(STATE_URL, timeout=2) as response:
        return json.load(response)


def fmt(num: float) -> str:
    return f"{num:8.2f}"


def main() -> int:
    try:
        state = fetch_state()
    except urllib.error.URLError as exc:
        print(f"Failed to fetch {STATE_URL}: {exc}", file=sys.stderr)
        return 1

    alignment = state.get("alignment", {})
    actual = state.get("actualSolution", {})
    hardware = state.get("hardware", {})
    motors = hardware.get("motors", [])

    print("Stewart Platform Debug")
    print(f"Mode: {state.get('mode')}  Link: {'ONLINE' if hardware.get('connected') else 'OFFLINE'}  Ready: {hardware.get('ready')}")
    print(
        "Pose:",
        "roll", fmt(state["pose"]["roll"]),
        "pitch", fmt(state["pose"]["pitch"]),
        "yaw", fmt(state["pose"]["yaw"]),
        "x", fmt(state["pose"]["x"]),
        "y", fmt(state["pose"]["y"]),
        "z", fmt(state["pose"]["z"]),
    )
    print(
        f"Calibration Z: {alignment.get('calibrationZ', 0):.3f}  "
        f"Z offset: {(state['pose']['z'] - alignment.get('calibrationZ', state['pose']['z'])):.3f}"
    )
    print(
        f"Actual FK: {'TRACKED' if actual.get('converged') else 'UNSOLVED'}  "
        f"Residual: {actual.get('residualNorm', 0):.3f}"
    )
    print()
    print("M   TServo   AServo   TMotor   AMotor    SErr    MErr  Sign     Zero    EncCnt  Turn")

    target_servo = alignment.get("targetServoAnglesDeg", [0.0] * 6)
    actual_servo = alignment.get("actualServoAnglesDeg", [0.0] * 6)
    target_motor = alignment.get("targetMotorAnglesDeg", [0.0] * 6)
    actual_motor = alignment.get("actualMotorAnglesDeg", [0.0] * 6)
    signs = alignment.get("motorSigns", [0] * 6)
    zeros = alignment.get("zeroOffsetsDeg", [0.0] * 6)

    for index in range(6):
        motor = motors[index] if index < len(motors) else {}
        s_err = actual_servo[index] - target_servo[index]
        m_err = actual_motor[index] - target_motor[index]
        print(
            f"{index + 1:<1} "
            f"{fmt(target_servo[index])} "
            f"{fmt(actual_servo[index])} "
            f"{fmt(target_motor[index])} "
            f"{fmt(actual_motor[index])} "
            f"{fmt(s_err)} "
            f"{fmt(m_err)} "
            f"{signs[index]:>5} "
            f"{fmt(zeros[index])} "
            f"{int(motor.get('encoderCount', 0)):>9} "
            f"{int(motor.get('singleTurnCount', 0)):>5}"
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
