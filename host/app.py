from __future__ import annotations

from dataclasses import asdict
import json
import math
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
import urllib.parse

import serial
import serial.tools.list_ports

from host.kinematics import Geometry, Pose, StewartKinematics


WEB_PORT = 8080
SERIAL_BAUD = 115200
STALE_TIMEOUT = 2.0
STATE_PATH = Path(__file__).resolve().parent.parent / ".runtime_state.json"


class HardwareBridge:
    def __init__(self) -> None:
        self.serial_port: serial.Serial | None = None
        self.serial_lock = threading.Lock()
        self.last_update = 0.0
        self.thread: threading.Thread | None = None
        self.running = False
        self.telemetry = self._empty_telemetry()

    def _empty_telemetry(self) -> dict[str, Any]:
        return {
            "ready": False,
            "connected": False,
            "stale": True,
            "port": None,
            "profile": {
                "positionSpeed": 180,
                "positionAccel": 12,
            },
            "motors": [
                {
                    "id": motor_id,
                    "on": False,
                    "deg": 0.0,
                    "targetDeg": 0.0,
                    "enabled": False,
                    "moving": False,
                    "modeCode": 0,
                    "mode": "UNKNOWN",
                    "workCurrentMa": 0,
                    "holdCurrentRatio": 0,
                    "holdCurrentPct": 0,
                    "configKnown": False,
                }
                for motor_id in range(1, 7)
            ],
        }

    def start(self) -> None:
        if self.thread:
            return
        self.running = True
        self.thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.thread.start()

    def _find_port(self) -> str | None:
        for port in serial.tools.list_ports.comports():
            description = f"{port.device} {port.description}".lower()
            if "usbserial" in description or "ch340" in description or "usb" in description:
                return port.device
        return None

    def _connect(self) -> None:
        port = self._find_port()
        if not port:
            return
        try:
            self.serial_port = serial.Serial(port, SERIAL_BAUD, timeout=1)
            self.telemetry["port"] = port
            self.telemetry["connected"] = True
        except Exception:
            self.serial_port = None
            self.telemetry["connected"] = False

    def _reader_loop(self) -> None:
        while self.running:
            with self.serial_lock:
                if self.serial_port is None or not self.serial_port.is_open:
                    self.telemetry["connected"] = False
                    self._connect()
            if self.serial_port is None:
                time.sleep(1)
                continue
            try:
                line = self.serial_port.readline().decode("utf-8", errors="ignore").strip()
                if not line or not line.startswith("{"):
                    continue
                data = json.loads(line)
                self.last_update = time.time()
                self.telemetry["stale"] = False
                if "ready" in data:
                    self.telemetry["ready"] = bool(data["ready"])
                if "motors" in data:
                    self.telemetry["motors"] = data["motors"]
                    # If telemetry is flowing, the firmware is already alive even if
                    # the one-shot {"ready":true} boot message was missed.
                    self.telemetry["ready"] = True
                if "profile" in data and isinstance(data["profile"], dict):
                    self.telemetry["profile"] = data["profile"]
            except (serial.SerialException, OSError):
                with self.serial_lock:
                    try:
                        if self.serial_port:
                            self.serial_port.close()
                    except Exception:
                        pass
                    self.serial_port = None
                self.telemetry["connected"] = False
                time.sleep(1)
            except json.JSONDecodeError:
                continue

    def state(self) -> dict[str, Any]:
        stale = (time.time() - self.last_update) > STALE_TIMEOUT if self.last_update else True
        telemetry = dict(self.telemetry)
        telemetry["stale"] = stale
        telemetry["connected"] = bool(self.serial_port and self.serial_port.is_open)
        return telemetry

    def send_line(self, line: str) -> bool:
        with self.serial_lock:
            if self.serial_port is None or not self.serial_port.is_open:
                return False
            try:
                self.serial_port.write(f"{line}\n".encode("utf-8"))
                return True
            except Exception:
                return False

    def enable_all(self, enable: bool) -> bool:
        return self.send_line(f"ENABLE:{1 if enable else 0}")

    def stop(self) -> bool:
        return self.send_line("STOP")

    def move_to_targets(self, angles_deg: list[float]) -> bool:
        payload = ",".join(f"{angle:.3f}" for angle in angles_deg)
        return self.send_line(f"MOVE:{payload}")

    def calibrate_all(self) -> bool:
        return self.send_line("CALIBRATE")

    def zero_motor(self, motor_id: int) -> bool:
        return self.send_line(f"ZERO:{motor_id}")

    def set_motion_profile(self, speed: int, accel: int) -> bool:
        return self.send_line(f"SET_PROFILE:{speed},{accel}")

    def set_work_current_all(self, current_ma: int) -> bool:
        return self.send_line(f"SET_CURRENT:{current_ma}")

    def set_hold_current_all(self, ratio: int) -> bool:
        return self.send_line(f"SET_HOLD_CURRENT:{ratio}")

    def set_protect_all(self, enable: bool) -> bool:
        return self.send_line(f"SET_PROTECT:{1 if enable else 0}")

    def release_protect_all(self) -> bool:
        return self.send_line("RELEASE_PROTECT")

    def read_protect(self) -> bool:
        return self.send_line("READ_PROTECT")

    def read_angle_error(self) -> bool:
        return self.send_line("READ_ANGLE_ERR")


class ControlState:
    def __init__(self) -> None:
        self.mode = "SIM"
        self.live_send = False
        self.motion_duration_ms = 1800
        self.pose = Pose(z=105.0)
        self.kinematics = StewartKinematics(Geometry())
        self.sequence: list[dict[str, float]] = []
        self.hardware = HardwareBridge()
        self.last_calibration: dict[str, Any] | None = None
        self.last_feedback: dict[str, Any] | None = None
        self.motion_state: dict[str, Any] = {"active": False, "progress": 1.0, "durationMs": self.motion_duration_ms}
        self.motor_settings: dict[str, int] = {
            "positionSpeed": 180,
            "positionAccel": 12,
            "workCurrentMa": 0,
            "holdCurrentRatio": -1,  # -1 = not set yet
        }
        self._state_lock = threading.RLock()
        self._trajectory_generation = 0
        self._load_runtime_state()
        self.hardware.start()
        self.last_solution = self.kinematics.solve(self.pose)

    def _zero_offsets_from_reference_motor_angles(self, motor_angles_deg: list[float]) -> list[float]:
        offsets: list[float] = []
        for index, sign in enumerate(self.kinematics.geometry.motor_signs):
            safe_sign = sign if sign != 0 else 1
            offsets.append(-motor_angles_deg[index] * safe_sign)
        return offsets

    def _reference_pose(self) -> Pose:
        calibration_pose = (self.last_calibration or {}).get("calibrationPose")
        if isinstance(calibration_pose, dict):
            return Pose(
                roll=float(calibration_pose.get("roll", 0.0)),
                pitch=float(calibration_pose.get("pitch", 0.0)),
                yaw=float(calibration_pose.get("yaw", 0.0)),
                x=float(calibration_pose.get("x", 0.0)),
                y=float(calibration_pose.get("y", 0.0)),
                z=float(calibration_pose.get("z", self.kinematics.geometry.home_z)),
            )
        return Pose(**self.pose.to_dict())

    def _all_up_reference_pose(self) -> Pose:
        return self.kinematics.calibration_pose()

    def _all_up_reference_motor_angles(self) -> list[float]:
        return [90.0] * 6

    def _wait_for_fresh_telemetry(self, timeout: float = 1.5) -> dict[str, Any]:
        deadline = time.time() + timeout
        latest = self.hardware.state()
        while time.time() < deadline:
            latest = self.hardware.state()
            if latest.get("connected") and not latest.get("stale") and latest.get("ready"):
                return latest
            time.sleep(0.05)
        return latest

    def _load_runtime_state(self) -> None:
        if not STATE_PATH.exists():
            self.kinematics.geometry.calibration_z = self.kinematics.calibration_pose().z
            self.kinematics.geometry.home_z = self.kinematics.operating_home_pose().z
            self.pose = Pose(z=self.kinematics.geometry.home_z)
            return
        try:
            payload = json.loads(STATE_PATH.read_text())
        except Exception:
            self.kinematics.geometry.calibration_z = self.kinematics.calibration_pose().z
            self.kinematics.geometry.home_z = self.kinematics.operating_home_pose().z
            self.pose = Pose(z=self.kinematics.geometry.home_z)
            return

        geometry = payload.get("geometry")
        if isinstance(geometry, dict):
            self.kinematics.update_geometry(geometry)

        if self.kinematics.geometry.calibration_z <= 0:
            self.kinematics.geometry.calibration_z = self.kinematics.calibration_pose().z

        # Migrate older state files that treated calibration height as home height.
        if self.kinematics.geometry.home_z <= 0 or abs(self.kinematics.geometry.home_z - self.kinematics.geometry.calibration_z) < 1e-6:
            self.kinematics.geometry.home_z = self.kinematics.operating_home_pose().z

        pose_data = payload.get("pose")
        if isinstance(pose_data, dict):
            self.pose = Pose(
                roll=float(pose_data.get("roll", 0.0)),
                pitch=float(pose_data.get("pitch", 0.0)),
                yaw=float(pose_data.get("yaw", 0.0)),
                x=float(pose_data.get("x", 0.0)),
                y=float(pose_data.get("y", 0.0)),
                z=float(pose_data.get("z", self.kinematics.geometry.home_z)),
            )
        else:
            self.pose = Pose(z=self.kinematics.geometry.home_z)

        calibration = payload.get("lastCalibration")
        if isinstance(calibration, dict):
            self.last_calibration = calibration
        feedback = payload.get("lastFeedback")
        if isinstance(feedback, dict):
            self.last_feedback = feedback
        motion_duration_ms = payload.get("motionDurationMs")
        if isinstance(motion_duration_ms, (int, float)):
            self.motion_duration_ms = max(100, int(motion_duration_ms))
        motor_settings = payload.get("motorSettings")
        if isinstance(motor_settings, dict):
            if "positionSpeed" in motor_settings:
                self.motor_settings["positionSpeed"] = max(0, min(3000, int(float(motor_settings["positionSpeed"]))))
            if "positionAccel" in motor_settings:
                self.motor_settings["positionAccel"] = max(0, min(255, int(float(motor_settings["positionAccel"]))))
            if "workCurrentMa" in motor_settings:
                self.motor_settings["workCurrentMa"] = max(0, min(3000, int(float(motor_settings["workCurrentMa"]))))
            if "holdCurrentRatio" in motor_settings:
                self.motor_settings["holdCurrentRatio"] = max(-1, min(8, int(float(motor_settings["holdCurrentRatio"]))))
        self.motion_state = {"active": False, "progress": 1.0, "durationMs": self.motion_duration_ms}

    def _persist_runtime_state(self) -> None:
        payload = {
            "geometry": self.kinematics.geometry.to_dict(),
            "pose": self.pose.to_dict(),
            "lastCalibration": self.last_calibration,
            "lastFeedback": self.last_feedback,
            "motionDurationMs": self.motion_duration_ms,
            "motorSettings": self.motor_settings,
        }
        STATE_PATH.write_text(json.dumps(payload, ensure_ascii=False, indent=2))

    def _ease_in_out(self, t: float) -> float:
        return t * t * (3.0 - 2.0 * t)

    def _interpolate_pose(self, start: Pose, end: Pose, alpha: float) -> Pose:
        return Pose(
            roll=start.roll + (end.roll - start.roll) * alpha,
            pitch=start.pitch + (end.pitch - start.pitch) * alpha,
            yaw=start.yaw + (end.yaw - start.yaw) * alpha,
            x=start.x + (end.x - start.x) * alpha,
            y=start.y + (end.y - start.y) * alpha,
            z=start.z + (end.z - start.z) * alpha,
        )

    def _start_pose_for_motion(self, target_pose: Pose) -> Pose:
        hardware_state = self.hardware.state()
        actual_solution = self.actual_solution(hardware_state)
        actual_pose = actual_solution.get("pose") if actual_solution.get("converged") else None
        if isinstance(actual_pose, dict):
            return Pose(
                roll=float(actual_pose.get("roll", target_pose.roll)),
                pitch=float(actual_pose.get("pitch", target_pose.pitch)),
                yaw=float(actual_pose.get("yaw", target_pose.yaw)),
                x=float(actual_pose.get("x", target_pose.x)),
                y=float(actual_pose.get("y", target_pose.y)),
                z=float(actual_pose.get("z", target_pose.z)),
            )
        return Pose(**target_pose.to_dict())

    def _run_motion(self, start_pose: Pose, target_pose: Pose, duration_ms: int, send_hardware: bool, generation: int) -> None:
        steps = max(2, int(math.ceil(duration_ms / 50.0)))
        start_time = time.time()
        with self._state_lock:
            self.motion_state = {"active": True, "progress": 0.0, "durationMs": duration_ms}
            self.last_feedback = {
                "type": "motion",
                "message": f"平滑移動中 {duration_ms / 1000:.1f}s",
                "timestamp": start_time,
            }
            self._persist_runtime_state()

        for step in range(1, steps + 1):
            with self._state_lock:
                if generation != self._trajectory_generation:
                    return
            t = step / steps
            alpha = self._ease_in_out(t)
            pose = self._interpolate_pose(start_pose, target_pose, alpha)
            solution = self.kinematics.solve(pose)
            if send_hardware and solution["reachable"]:
                self.hardware.move_to_targets(solution["servo_angles_deg"])
            with self._state_lock:
                if generation != self._trajectory_generation:
                    return
                self.pose = pose
                self.last_solution = solution
                self.motion_state = {"active": True, "progress": t, "durationMs": duration_ms}
            if step < steps:
                time.sleep(duration_ms / steps / 1000.0)

        with self._state_lock:
            if generation != self._trajectory_generation:
                return
            self.pose = target_pose
            self.last_solution = self.kinematics.solve(self.pose)
            self.motion_state = {"active": False, "progress": 1.0, "durationMs": duration_ms}
            self.last_feedback = {
                "type": "motion_done",
                "message": f"平滑移動完成 {duration_ms / 1000:.1f}s",
                "timestamp": time.time(),
            }
            self._persist_runtime_state()

    def _queue_motion(self, target_pose: Pose, duration_ms: int, send_hardware: bool) -> None:
        with self._state_lock:
            self._trajectory_generation += 1
            generation = self._trajectory_generation
            start_pose = self._start_pose_for_motion(target_pose)
        worker = threading.Thread(
            target=self._run_motion,
            args=(start_pose, target_pose, duration_ms, send_hardware, generation),
            daemon=True,
        )
        worker.start()

    def actual_solution(self, telemetry: dict[str, Any]) -> dict[str, Any]:
        servo_angles = [float(motor.get("deg", 0.0)) for motor in telemetry.get("motors", [])[:6]]
        if len(servo_angles) < 6:
            servo_angles.extend([0.0] * (6 - len(servo_angles)))
        return self.kinematics.solve_from_servo_angles(servo_angles, initial_pose=self.pose)

    def snapshot(self) -> dict[str, Any]:
        hardware_state = self.hardware.state()
        actual_solution = self.actual_solution(hardware_state)
        return {
            "mode": self.mode,
            "liveSend": self.live_send,
            "pose": self.pose.to_dict(),
            "geometry": self.kinematics.geometry.to_dict(),
            "solution": self.last_solution,
            "actualSolution": actual_solution,
            "hardware": hardware_state,
            "motorSettings": self.motor_settings,
            "alignment": {
                "calibrationZ": self.kinematics.geometry.calibration_z,
                "homeZ": self.kinematics.geometry.home_z,
                "targetServoAnglesDeg": self.last_solution["servo_angles_deg"],
                "targetMotorAnglesDeg": self.last_solution["motor_angles_deg"],
                "actualServoAnglesDeg": [float(motor.get("deg", 0.0)) for motor in hardware_state.get("motors", [])[:6]],
                "actualMotorAnglesDeg": actual_solution.get("motor_angles_deg", [0.0] * 6),
                "zeroOffsetsDeg": self.kinematics.geometry.zero_offsets_deg,
                "motorSigns": self.kinematics.geometry.motor_signs,
            },
            "calibration": self.last_calibration,
            "feedback": self.last_feedback,
            "motion": self.motion_state,
            "sequence": self.sequence,
        }

    def update_pose(self, payload: dict[str, Any], apply_hardware: bool = False) -> dict[str, Any]:
        duration_ms = int(payload.get("durationMs", self.motion_duration_ms))
        target_pose = Pose(**self.pose.to_dict())
        for key in ("roll", "pitch", "yaw", "x", "y", "z"):
            if key in payload:
                setattr(target_pose, key, float(payload[key]))
        self.pose = target_pose
        self.last_solution = self.kinematics.solve(self.pose)
        should_send = apply_hardware or (self.live_send and self.mode == "SIM+HW")
        if apply_hardware and self.last_solution["reachable"]:
            self.motion_duration_ms = max(100, duration_ms)
            self._queue_motion(target_pose, self.motion_duration_ms, True)
            self.last_feedback = {
                "type": "motion_queue",
                "message": f"已排入平滑移動 {self.motion_duration_ms / 1000:.1f}s",
                "timestamp": time.time(),
            }
        elif should_send and self.last_solution["reachable"]:
            self.hardware.move_to_targets(self.last_solution["servo_angles_deg"])
        self._persist_runtime_state()
        return self.snapshot()

    def update_geometry(self, payload: dict[str, Any]) -> dict[str, Any]:
        normalized = {}
        for key, value in payload.items():
            if key in {"stepper_plane_angles", "motor_signs", "zero_offsets_deg"}:
                normalized[key] = [float(item) for item in value]
            elif key == "servo_pulses_per_rev":
                normalized[key] = int(value)
            else:
                normalized[key] = float(value)
        self.kinematics.update_geometry(normalized)
        self.last_solution = self.kinematics.solve(self.pose)
        self._persist_runtime_state()
        return self.snapshot()

    def set_mode(self, mode: str, live_send: bool) -> dict[str, Any]:
        self.mode = mode
        self.live_send = live_send
        self._persist_runtime_state()
        return self.snapshot()

    def update_settings(self, payload: dict[str, Any]) -> dict[str, Any]:
        duration = payload.get("motionDurationMs")
        if duration is not None:
            self.motion_duration_ms = max(100, int(float(duration)))
            self.motion_state["durationMs"] = self.motion_duration_ms
            self.last_feedback = {
                "type": "settings",
                "message": f"平滑時間已設為 {self.motion_duration_ms / 1000:.1f}s",
                "timestamp": time.time(),
            }
        changed_parts: list[str] = []
        position_speed = payload.get("positionSpeed")
        position_accel = payload.get("positionAccel")
        work_current_ma = payload.get("workCurrentMa")
        if position_speed is not None or position_accel is not None:
            speed = self.motor_settings["positionSpeed"] if position_speed is None else max(0, min(3000, int(float(position_speed))))
            accel = self.motor_settings["positionAccel"] if position_accel is None else max(0, min(255, int(float(position_accel))))
            self.motor_settings["positionSpeed"] = speed
            self.motor_settings["positionAccel"] = accel
            self.hardware.set_motion_profile(speed, accel)
            changed_parts.append(f"profile={speed}/{accel}")
        if work_current_ma is not None:
            current_ma = max(0, min(3000, int(float(work_current_ma))))
            self.motor_settings["workCurrentMa"] = current_ma
            self.hardware.set_work_current_all(current_ma)
            changed_parts.append(f"ma={current_ma}")
        hold_current_ratio = payload.get("holdCurrentRatio")
        if hold_current_ratio is not None:
            ratio = max(0, min(8, int(float(hold_current_ratio))))
            self.motor_settings["holdCurrentRatio"] = ratio
            self.hardware.set_hold_current_all(ratio)
            hold_pct = ratio * 10 + 10
            changed_parts.append(f"hold={hold_pct}%")
        protect_enable = payload.get("protectEnable")
        if protect_enable is not None:
            self.hardware.set_protect_all(bool(protect_enable))
            changed_parts.append(f"protect={'on' if protect_enable else 'off'}")
        if changed_parts:
            self.last_feedback = {
                "type": "motor_settings",
                "message": "已更新馬達設定 " + " ".join(changed_parts),
                "timestamp": time.time(),
            }
        self._persist_runtime_state()
        return self.snapshot()

    def execute_command(self, command: str, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        payload = payload or {}
        if command == "enable_all":
            self.hardware.enable_all(True)
            self.last_feedback = {"type": "enable_all", "message": "已使能全部馬達", "timestamp": time.time()}
        elif command == "disable_all":
            self.hardware.enable_all(False)
            self.last_feedback = {"type": "disable_all", "message": "已失能全部馬達，可手動調整", "timestamp": time.time()}
        elif command == "stop":
            self.hardware.stop()
            self.last_feedback = {"type": "stop", "message": "已送出急停", "timestamp": time.time()}
        elif command == "release_protect":
            self.hardware.release_protect_all()
            self.last_feedback = {"type": "release_protect", "message": "已解除堵轉保護", "timestamp": time.time()}
        elif command == "read_protect":
            self.hardware.read_protect()
            self.last_feedback = {"type": "read_protect", "message": "已請求讀取保護狀態（查看 serial log）", "timestamp": time.time()}
        elif command == "read_angle_error":
            self.hardware.read_angle_error()
            self.last_feedback = {"type": "read_angle_error", "message": "已請求讀取角度誤差（查看 serial log）", "timestamp": time.time()}
        elif command == "calibrate":
            before = self._wait_for_fresh_telemetry()
            before_actual = self.actual_solution(before)
            if before_actual.get("converged") and isinstance(before_actual.get("pose"), dict):
                actual_pose = before_actual["pose"]
                reference_pose = Pose(
                    roll=float(actual_pose.get("roll", self.pose.roll)),
                    pitch=float(actual_pose.get("pitch", self.pose.pitch)),
                    yaw=float(actual_pose.get("yaw", self.pose.yaw)),
                    x=float(actual_pose.get("x", self.pose.x)),
                    y=float(actual_pose.get("y", self.pose.y)),
                    z=float(actual_pose.get("z", self.pose.z)),
                )
                reference_motor_angles = [float(angle) for angle in before_actual.get("motor_angles_deg", [0.0] * 6)]
            else:
                reference_pose = Pose(**self.pose.to_dict())
                reference_motor_angles = [float(angle) for angle in self.last_solution.get("motor_angles_deg", [0.0] * 6)]
            self.hardware.calibrate_all()
            time.sleep(0.35)
            after = self._wait_for_fresh_telemetry()

            self.kinematics.geometry.zero_offsets_deg = self._zero_offsets_from_reference_motor_angles(reference_motor_angles)
            self.kinematics.geometry.calibration_z = reference_pose.z
            self.kinematics.geometry.home_z = reference_pose.z
            self.pose = reference_pose
            self.last_solution = self.kinematics.solve(self.pose)
            self.last_calibration = {
                "timestamp": time.time(),
                "calibrationPose": reference_pose.to_dict(),
                "referenceMotorAnglesDeg": reference_motor_angles,
                "beforeServoDeg": [float(motor.get("deg", 0.0)) for motor in before.get("motors", [])[:6]],
                "beforeRawDeg": [float(motor.get("rawDeg", 0.0)) for motor in before.get("motors", [])[:6]],
                "afterServoDeg": [float(motor.get("deg", 0.0)) for motor in after.get("motors", [])[:6]],
                "afterRawDeg": [float(motor.get("rawDeg", 0.0)) for motor in after.get("motors", [])[:6]],
            }
            self.last_feedback = {
                "type": "calibrate_all",
                "message": "已將當前姿態設為校正基準",
                "timestamp": time.time(),
            }
        elif command == "all_up_reference":
            before = self._wait_for_fresh_telemetry()
            self.hardware.calibrate_all()
            time.sleep(0.35)
            after = self._wait_for_fresh_telemetry()
            reference_pose = self._all_up_reference_pose()
            reference_motor_angles = self._all_up_reference_motor_angles()
            self.kinematics.geometry.zero_offsets_deg = self._zero_offsets_from_reference_motor_angles(reference_motor_angles)
            self.kinematics.geometry.calibration_z = reference_pose.z
            self.kinematics.geometry.home_z = reference_pose.z
            self.pose = reference_pose
            self.last_solution = self.kinematics.solve(self.pose)
            self.last_calibration = {
                "timestamp": time.time(),
                "calibrationPose": reference_pose.to_dict(),
                "referenceMotorAnglesDeg": reference_motor_angles,
                "beforeServoDeg": [float(motor.get("deg", 0.0)) for motor in before.get("motors", [])[:6]],
                "beforeRawDeg": [float(motor.get("rawDeg", 0.0)) for motor in before.get("motors", [])[:6]],
                "afterServoDeg": [float(motor.get("deg", 0.0)) for motor in after.get("motors", [])[:6]],
                "afterRawDeg": [float(motor.get("rawDeg", 0.0)) for motor in after.get("motors", [])[:6]],
            }
            self.last_feedback = {
                "type": "all_up_reference",
                "message": "已將全部向上設為校正基準",
                "timestamp": time.time(),
            }
        elif command == "zero_motor":
            motor_id = int(payload.get("motorId", 0))
            motors_before = self._wait_for_fresh_telemetry().get("motors", [])
            if 1 <= motor_id <= 6 and len(motors_before) >= motor_id:
                before_seq = int(motors_before[motor_id - 1].get("zeroSeq", 0))
                self.hardware.zero_motor(motor_id)
                time.sleep(0.2)
                motors_after = self._wait_for_fresh_telemetry().get("motors", [])
                after_seq = int(motors_after[motor_id - 1].get("zeroSeq", 0)) if len(motors_after) >= motor_id else before_seq
                self.last_feedback = {
                    "type": "zero_motor",
                    "motorId": motor_id,
                    "succeeded": after_seq > before_seq,
                    "message": f"M{motor_id} 校正完成" if after_seq > before_seq else f"M{motor_id} 校正未確認",
                    "timestamp": time.time(),
                }
        elif command == "apply_pose":
            if self.last_solution["reachable"]:
                self.hardware.move_to_targets(self.last_solution["servo_angles_deg"])
                self.last_feedback = {"type": "apply_pose", "message": "已送出姿態目標", "timestamp": time.time()}
        elif command == "record_keyframe":
            self.sequence.append(self.pose.to_dict())
        elif command == "clear_keyframes":
            self.sequence = []
        elif command == "home":
            self.pose = self._reference_pose()
            self.kinematics.geometry.home_z = self.pose.z
            self.kinematics.geometry.calibration_z = self.pose.z
            self.last_solution = self.kinematics.solve(self.pose)
            self.last_feedback = {"type": "home", "message": "已回到校正基準", "timestamp": time.time()}
        self._persist_runtime_state()
        return self.snapshot()


APP_STATE = ControlState()
WEB_ROOT = Path(__file__).resolve().parent.parent / "web"


class AppHandler(BaseHTTPRequestHandler):
    def do_GET(self) -> None:
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == "/api/state":
            self._json(APP_STATE.snapshot())
            return
        if parsed.path == "/" or parsed.path == "":
            self._serve_static("index.html", "text/html; charset=utf-8")
            return
        self._serve_path(parsed.path.lstrip("/"))

    def do_POST(self) -> None:
        parsed = urllib.parse.urlparse(self.path)
        data = self._read_json()

        if parsed.path == "/api/pose":
            apply_hardware = bool(data.get("applyHardware", False))
            self._json(APP_STATE.update_pose(data.get("pose", {}), apply_hardware))
            return
        if parsed.path == "/api/geometry":
            self._json(APP_STATE.update_geometry(data))
            return
        if parsed.path == "/api/settings":
            self._json(APP_STATE.update_settings(data))
            return
        if parsed.path == "/api/mode":
            self._json(APP_STATE.set_mode(data.get("mode", "SIM"), bool(data.get("liveSend", False))))
            return
        if parsed.path == "/api/command":
            self._json(APP_STATE.execute_command(data.get("command", ""), data))
            return

        self.send_error(HTTPStatus.NOT_FOUND)

    def _read_json(self) -> dict[str, Any]:
        content_length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(content_length) if content_length else b"{}"
        return json.loads(raw.decode("utf-8"))

    def _json(self, payload: dict[str, Any], status: int = 200) -> None:
        encoded = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)

    def _serve_path(self, relative_path: str) -> None:
        target = (WEB_ROOT / relative_path).resolve()
        if WEB_ROOT not in target.parents and target != WEB_ROOT:
            self.send_error(HTTPStatus.FORBIDDEN)
            return
        if not target.exists() or target.is_dir():
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        content_type = "text/plain; charset=utf-8"
        if target.suffix == ".css":
            content_type = "text/css; charset=utf-8"
        elif target.suffix == ".js":
            content_type = "application/javascript; charset=utf-8"
        elif target.suffix == ".html":
            content_type = "text/html; charset=utf-8"
        elif target.suffix == ".json":
            content_type = "application/json"
        self._serve_static(relative_path, content_type)

    def _serve_static(self, relative_path: str, content_type: str) -> None:
        path = WEB_ROOT / relative_path
        data = path.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, fmt: str, *args: Any) -> None:
        return


def run_server() -> None:
    httpd = ThreadingHTTPServer(("", WEB_PORT), AppHandler)
    print(f"Control panel: http://localhost:{WEB_PORT}")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down...")


if __name__ == "__main__":
    run_server()
