#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.error
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


def wait_for_hardware(timeout: float = 5.0) -> dict:
    deadline = time.time() + timeout
    latest = {}
    while time.time() < deadline:
        latest = api_get("/api/state")
        hw = latest["hardware"]
        if hw["connected"] and not hw["stale"] and hw["ready"]:
            return latest
        time.sleep(0.1)
    raise RuntimeError("hardware not ready")


def print_state(state: dict) -> None:
    actual = state["actualSolution"]
    alignment = state["alignment"]
    print(
        f"Mode={state['mode']} Link={'ONLINE' if state['hardware']['connected'] else 'OFFLINE'} "
        f"Ready={state['hardware']['ready']} PoseZ={state['pose']['z']:.3f} "
        f"CalZ={alignment['calibrationZ']:.3f} Residual={actual['residualNorm']:.3f} "
        f"ActualFK={'TRACKED' if actual['converged'] else 'UNSOLVED'}"
    )
    for index, motor in enumerate(state["hardware"]["motors"], start=1):
        target = state["solution"]["servo_angles_deg"][index - 1]
        actual_deg = float(motor.get("deg", 0.0))
        mode = motor.get("mode", "UNKNOWN")
        work_current = int(motor.get("workCurrentMa", 0))
        hold_pct = int(motor.get("holdCurrentPct", 0))
        config_known = bool(motor.get("configKnown", False))
        mode_text = mode if config_known else "PENDING"
        ma_text = f"{work_current:4d}mA" if config_known else " ----"
        hold_text = f"{hold_pct:2d}%" if config_known else " --"
        print(
            f"M{index}: target={target:8.2f} actual={actual_deg:8.2f} "
            f"error={actual_deg - target:8.2f} raw={float(motor.get('rawDeg', 0.0)):9.2f} "
            f"mode={mode_text:>8} ma={ma_text} hold={hold_text} cfg={'Y' if config_known else 'N'}"
        )


def command_status(_: argparse.Namespace) -> int:
    print_state(api_get("/api/state"))
    return 0


def command_calibrate(args: argparse.Namespace) -> int:
    wait_for_hardware()
    api_post("/api/mode", {"mode": "SIM+HW", "liveSend": False})
    api_post("/api/command", {"command": "enable_all"})
    state = api_post("/api/command", {"command": "calibrate"})

    deadline = time.time() + args.timeout
    while time.time() < deadline:
        state = api_get("/api/state")
        actual = state["actualSolution"]
        servo_errors = [abs(float(m["deg"])) for m in state["hardware"]["motors"]]
        if actual["converged"] and actual["residualNorm"] < 5.0 and max(servo_errors, default=999.0) <= args.tolerance:
            break
        time.sleep(0.15)

    print_state(state)
    calibration = state.get("calibration") or {}
    if calibration:
        print("\nCalibration snapshot:")
        print(json.dumps(calibration, ensure_ascii=False, indent=2))
    return 0


def command_validate(args: argparse.Namespace) -> int:
    state = wait_for_hardware()
    cal_z = state["alignment"]["calibrationZ"]
    api_post("/api/mode", {"mode": "SIM+HW", "liveSend": False})
    api_post("/api/command", {"command": "enable_all"})

    offsets = [0.0]
    for depth in args.offsets:
        offsets.append(-abs(depth))
    offsets.extend(reversed(offsets[:-1]))

    for offset in offsets:
        target_z = cal_z + offset
        api_post("/api/pose", {"pose": {"z": target_z}, "applyHardware": True})
        time.sleep(args.wait)
        state = api_get("/api/state")
        errors = [round(float(m["deg"]) - float(m["targetDeg"]), 2) for m in state["hardware"]["motors"]]
        print(f"offset={offset:>6.2f} max_err={max(abs(err) for err in errors):>5.2f} errs={errors}")
    return 0


def command_goto_cal(_: argparse.Namespace) -> int:
    state = wait_for_hardware()
    cal_z = state["alignment"]["calibrationZ"]
    api_post("/api/mode", {"mode": "SIM+HW", "liveSend": False})
    api_post("/api/command", {"command": "enable_all"})
    api_post("/api/pose", {"pose": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "x": 0.0, "y": 0.0, "z": cal_z}, "applyHardware": True})
    time.sleep(2.0)
    print_state(api_get("/api/state"))
    return 0


def command_watch(args: argparse.Namespace) -> int:
    end_time = None if args.seconds <= 0 else time.time() + args.seconds
    try:
        while True:
            state = api_get("/api/state")
            actual = state["actualSolution"]
            motors = state["hardware"]["motors"]
            servo = [float(m["deg"]) for m in motors]
            raw = [float(m.get("rawDeg", 0.0)) for m in motors]
            print(
                f"t={time.strftime('%H:%M:%S')} "
                f"poseZ={state['pose']['z']:.3f} "
                f"actualZ={actual.get('pose', {}).get('z', 0.0):.3f} "
                f"res={actual.get('residualNorm', 0.0):.3f} "
                f"servo={[round(v, 2) for v in servo]} "
                f"raw={[round(v, 1) for v in raw]}"
            )
            if end_time is not None and time.time() >= end_time:
                break
            time.sleep(args.interval)
    except KeyboardInterrupt:
        return 0
    return 0


def command_recalibrate(args: argparse.Namespace) -> int:
    wait_for_hardware()
    print("[1/3] calibrating from current manual upright pose")
    command_calibrate(argparse.Namespace(timeout=args.timeout, tolerance=args.tolerance))
    print("\n[2/3] moving back to calibrated pose")
    command_goto_cal(argparse.Namespace())
    print("\n[3/3] validating Z sweep")
    command_validate(argparse.Namespace(offsets=args.offsets, wait=args.wait))
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Stewart platform operations helper")
    sub = parser.add_subparsers(dest="command", required=True)

    status = sub.add_parser("status", help="Print current operational state")
    status.set_defaults(func=command_status)

    calibrate = sub.add_parser("calibrate", help="Run hardware-first calibration and wait for stability")
    calibrate.add_argument("--timeout", type=float, default=3.0)
    calibrate.add_argument("--tolerance", type=float, default=2.0)
    calibrate.set_defaults(func=command_calibrate)

    validate = sub.add_parser("validate", help="Run a small Z sweep from calibration")
    validate.add_argument("--offsets", type=float, nargs="*", default=[5.0, 10.0, 15.0, 20.0])
    validate.add_argument("--wait", type=float, default=2.0)
    validate.set_defaults(func=command_validate)

    goto_cal = sub.add_parser("goto-cal", help="Move the machine to the calibrated upright pose")
    goto_cal.set_defaults(func=command_goto_cal)

    watch = sub.add_parser("watch", help="Continuously print live state while manually trimming the machine")
    watch.add_argument("--interval", type=float, default=0.25)
    watch.add_argument("--seconds", type=float, default=0.0, help="0 means run until interrupted")
    watch.set_defaults(func=command_watch)

    recal = sub.add_parser("recalibrate", help="Run calibrate -> goto-cal -> validate in order")
    recal.add_argument("--timeout", type=float, default=3.0)
    recal.add_argument("--tolerance", type=float, default=2.0)
    recal.add_argument("--offsets", type=float, nargs="*", default=[5.0, 10.0, 15.0])
    recal.add_argument("--wait", type=float, default=2.2)
    recal.set_defaults(func=command_recalibrate)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        return args.func(args)
    except (urllib.error.URLError, RuntimeError) as exc:
        print(str(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
