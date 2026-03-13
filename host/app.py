from __future__ import annotations

from dataclasses import asdict
import json
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
            "motors": [
                {
                    "id": motor_id,
                    "on": False,
                    "deg": 0.0,
                    "targetDeg": 0.0,
                    "enabled": False,
                    "moving": False,
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


class ControlState:
    def __init__(self) -> None:
        self.mode = "SIM"
        self.live_send = False
        self.pose = Pose(z=105.0)
        self.kinematics = StewartKinematics(Geometry())
        self.sequence: list[dict[str, float]] = []
        self.hardware = HardwareBridge()
        self.hardware.start()
        self.last_solution = self.kinematics.solve(self.pose)
        self.last_calibration: dict[str, Any] | None = None

    def _calibration_zero_offsets(self) -> list[float]:
        return [-90.0 * sign for sign in self.kinematics.geometry.motor_signs]

    def _wait_for_fresh_telemetry(self, timeout: float = 1.5) -> dict[str, Any]:
        deadline = time.time() + timeout
        latest = self.hardware.state()
        while time.time() < deadline:
            latest = self.hardware.state()
            if latest.get("connected") and not latest.get("stale") and latest.get("ready"):
                return latest
            time.sleep(0.05)
        return latest

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
            "alignment": {
                "calibrationZ": self.kinematics.geometry.home_z,
                "targetServoAnglesDeg": self.last_solution["servo_angles_deg"],
                "targetMotorAnglesDeg": self.last_solution["motor_angles_deg"],
                "actualServoAnglesDeg": [float(motor.get("deg", 0.0)) for motor in hardware_state.get("motors", [])[:6]],
                "actualMotorAnglesDeg": actual_solution.get("motor_angles_deg", [0.0] * 6),
                "zeroOffsetsDeg": self.kinematics.geometry.zero_offsets_deg,
                "motorSigns": self.kinematics.geometry.motor_signs,
            },
            "calibration": self.last_calibration,
            "sequence": self.sequence,
        }

    def update_pose(self, payload: dict[str, Any], apply_hardware: bool = False) -> dict[str, Any]:
        for key in ("roll", "pitch", "yaw", "x", "y", "z"):
            if key in payload:
                setattr(self.pose, key, float(payload[key]))
        self.last_solution = self.kinematics.solve(self.pose)
        should_send = apply_hardware or (self.live_send and self.mode == "SIM+HW")
        if should_send and self.last_solution["reachable"]:
            self.hardware.move_to_targets(self.last_solution["servo_angles_deg"])
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
        return self.snapshot()

    def set_mode(self, mode: str, live_send: bool) -> dict[str, Any]:
        self.mode = mode
        self.live_send = live_send
        return self.snapshot()

    def execute_command(self, command: str, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        payload = payload or {}
        if command == "enable_all":
            self.hardware.enable_all(True)
        elif command == "disable_all":
            self.hardware.enable_all(False)
        elif command == "stop":
            self.hardware.stop()
        elif command == "calibrate":
            before = self._wait_for_fresh_telemetry()
            self.hardware.calibrate_all()
            time.sleep(0.35)
            after = self._wait_for_fresh_telemetry()

            calibration_pose = self.kinematics.calibration_pose()
            self.kinematics.geometry.zero_offsets_deg = self._calibration_zero_offsets()
            self.kinematics.geometry.home_z = calibration_pose.z
            self.pose = calibration_pose
            self.last_solution = self.kinematics.solve(self.pose)
            self.last_calibration = {
                "timestamp": time.time(),
                "calibrationPose": calibration_pose.to_dict(),
                "beforeServoDeg": [float(motor.get("deg", 0.0)) for motor in before.get("motors", [])[:6]],
                "beforeRawDeg": [float(motor.get("rawDeg", 0.0)) for motor in before.get("motors", [])[:6]],
                "afterServoDeg": [float(motor.get("deg", 0.0)) for motor in after.get("motors", [])[:6]],
                "afterRawDeg": [float(motor.get("rawDeg", 0.0)) for motor in after.get("motors", [])[:6]],
            }
        elif command == "apply_pose":
            if self.last_solution["reachable"]:
                self.hardware.move_to_targets(self.last_solution["servo_angles_deg"])
        elif command == "record_keyframe":
            self.sequence.append(self.pose.to_dict())
        elif command == "clear_keyframes":
            self.sequence = []
        elif command == "home":
            self.pose = Pose(z=self.kinematics.geometry.home_z)
            self.last_solution = self.kinematics.solve(self.pose)
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
