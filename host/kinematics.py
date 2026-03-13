from __future__ import annotations

from dataclasses import asdict, dataclass, field
import math
from typing import List


def _mat_mul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [
        [
            sum(a[row][k] * b[k][col] for k in range(3))
            for col in range(3)
        ]
        for row in range(3)
    ]


def _mat_vec_mul(m: list[list[float]], v: list[float]) -> list[float]:
    return [sum(m[row][k] * v[k] for k in range(3)) for row in range(3)]


def _solve_linear_system(a: list[list[float]], b: list[float]) -> list[float]:
    n = len(b)
    aug = [row[:] + [b[idx]] for idx, row in enumerate(a)]

    for col in range(n):
        pivot = max(range(col, n), key=lambda row: abs(aug[row][col]))
        if abs(aug[pivot][col]) < 1e-9:
            raise ValueError("Singular system")
        aug[col], aug[pivot] = aug[pivot], aug[col]

        scale = aug[col][col]
        for k in range(col, n + 1):
            aug[col][k] /= scale

        for row in range(n):
            if row == col:
                continue
            factor = aug[row][col]
            for k in range(col, n + 1):
                aug[row][k] -= factor * aug[col][k]

    return [aug[row][n] for row in range(n)]


@dataclass
class Geometry:
    base_radius: float = 152.0
    base_angle: float = 18.92
    platform_radius: float = 103.0
    platform_angle: float = 28.07
    lower_leg: float = 65.0
    upper_leg: float = 165.0
    home_z: float = 105.0
    stepper_plane_angles: List[float] = field(
        default_factory=lambda: [-90.0, 90.0, 30.0, 210.0, -210.0, -30.0]
    )
    motor_signs: List[int] = field(default_factory=lambda: [1, -1, 1, -1, 1, -1])
    zero_offsets_deg: List[float] = field(default_factory=lambda: [0.0] * 6)
    servo_pulses_per_rev: int = 6400
    default_speed: int = 180
    default_accel: int = 12

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass
class Pose:
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    x: float = 0.0
    y: float = 0.0
    z: float = 105.0

    def to_dict(self) -> dict:
        return asdict(self)


class StewartKinematics:
    def __init__(self, geometry: Geometry | None = None) -> None:
        self.geometry = geometry or Geometry()

    def update_geometry(self, data: dict) -> None:
        for key, value in data.items():
            if hasattr(self.geometry, key):
                setattr(self.geometry, key, value)

    def base_points(self) -> list[list[float]]:
        g = self.geometry
        return self._hex_points(g.base_radius, g.base_angle)

    def platform_points(self) -> list[list[float]]:
        g = self.geometry
        return self._hex_points(g.platform_radius, g.platform_angle)

    def _hex_points(self, radius: float, pair_angle: float) -> list[list[float]]:
        angles = [
            -pair_angle / 2,
            pair_angle / 2,
            -pair_angle / 2 + 120,
            pair_angle / 2 + 120,
            -pair_angle / 2 - 120,
            pair_angle / 2 - 120,
        ]
        points: list[list[float]] = []
        for angle in angles:
            rad = math.radians(angle)
            points.append([radius * math.cos(rad), radius * math.sin(rad), 0.0])
        return points

    def rotation_matrix(self, pose: Pose) -> list[list[float]]:
        roll = math.radians(pose.roll)
        pitch = math.radians(pose.pitch)
        yaw = math.radians(pose.yaw)

        rz = [
            [math.cos(yaw), -math.sin(yaw), 0.0],
            [math.sin(yaw), math.cos(yaw), 0.0],
            [0.0, 0.0, 1.0],
        ]
        ry = [
            [math.cos(pitch), 0.0, math.sin(pitch)],
            [0.0, 1.0, 0.0],
            [-math.sin(pitch), 0.0, math.cos(pitch)],
        ]
        rx = [
            [1.0, 0.0, 0.0],
            [0.0, math.cos(roll), -math.sin(roll)],
            [0.0, math.sin(roll), math.cos(roll)],
        ]
        return _mat_mul(_mat_mul(rz, ry), rx)

    def calibration_pose(self) -> Pose:
        g = self.geometry
        base = self.base_points()
        platform = self.platform_points()

        z_values: list[float] = []
        for index in range(6):
            dx = platform[index][0] - base[index][0]
            dy = platform[index][1] - base[index][1]
            planar_sq = dx * dx + dy * dy
            rod_vertical_sq = g.upper_leg * g.upper_leg - planar_sq
            if rod_vertical_sq <= 0:
                raise ValueError("Geometry cannot reach vertical-up calibration pose")
            z_values.append(g.lower_leg + math.sqrt(rod_vertical_sq))

        calibration_z = sum(z_values) / len(z_values)
        return Pose(roll=0.0, pitch=0.0, yaw=0.0, x=0.0, y=0.0, z=calibration_z)

    def servo_to_motor_angles(self, servo_angles_deg: list[float]) -> list[float]:
        g = self.geometry
        motor_angles: list[float] = []
        for index in range(6):
            sign = g.motor_signs[index] if g.motor_signs[index] != 0 else 1
            motor_angles.append((servo_angles_deg[index] - g.zero_offsets_deg[index]) / sign)
        return motor_angles

    def crank_points_from_motor_angles(self, motor_angles_deg: list[float]) -> list[list[float]]:
        g = self.geometry
        base = self.base_points()
        crank_points: list[list[float]] = []
        for index in range(6):
            plane = math.radians(g.stepper_plane_angles[index])
            angle = math.radians(motor_angles_deg[index])
            b = base[index]
            crank_points.append(
                [
                    g.lower_leg * math.cos(angle) * math.cos(plane) + b[0],
                    g.lower_leg * math.cos(angle) * math.sin(plane) + b[1],
                    g.lower_leg * math.sin(angle) + b[2],
                ]
            )
        return crank_points

    def _pose_residuals(self, pose: Pose, crank_points: list[list[float]]) -> tuple[list[float], list[list[float]]]:
        rotation = self.rotation_matrix(pose)
        platform_local = self.platform_points()
        world_points: list[list[float]] = []
        residuals: list[float] = []
        for index in range(6):
            p_world = _mat_vec_mul(rotation, platform_local[index])
            q = [p_world[i] + [pose.x, pose.y, pose.z][i] for i in range(3)]
            world_points.append(q)
            diff = [q[axis] - crank_points[index][axis] for axis in range(3)]
            residuals.append(sum(component * component for component in diff) - self.geometry.upper_leg**2)
        return residuals, world_points

    def solve_pose_from_motor_angles(
        self,
        motor_angles_deg: list[float],
        initial_pose: Pose | None = None,
        iterations: int = 12,
    ) -> dict:
        crank_points = self.crank_points_from_motor_angles(motor_angles_deg)
        pose = initial_pose or self.calibration_pose()
        pose = Pose(**pose.to_dict())

        converged = False
        for _ in range(iterations):
            residuals, platform_world = self._pose_residuals(pose, crank_points)
            residual_norm = math.sqrt(sum(item * item for item in residuals))
            if residual_norm < 1e-4:
                converged = True
                break

            deltas = [0.05, 0.05, 0.05, 0.01, 0.01, 0.01]
            params = [pose.roll, pose.pitch, pose.yaw, pose.x, pose.y, pose.z]
            jacobian: list[list[float]] = []
            for idx, delta in enumerate(deltas):
                shifted = params[:]
                shifted[idx] += delta
                shifted_pose = Pose(*shifted)
                shifted_residuals, _ = self._pose_residuals(shifted_pose, crank_points)
                jacobian.append([(shifted_residuals[row] - residuals[row]) / delta for row in range(6)])

            # Build JTJ and JTr for Gauss-Newton
            jtj = [[0.0] * 6 for _ in range(6)]
            jtr = [0.0] * 6
            for row in range(6):
                for col in range(6):
                    jtj[row][col] = sum(jacobian[row][k] * jacobian[col][k] for k in range(6))
                jtr[row] = sum(jacobian[row][k] * residuals[k] for k in range(6))

            try:
                step = _solve_linear_system(jtj, [-value for value in jtr])
            except ValueError:
                break

            pose.roll += step[0]
            pose.pitch += step[1]
            pose.yaw += step[2]
            pose.x += step[3]
            pose.y += step[4]
            pose.z += step[5]

        residuals, platform_world = self._pose_residuals(pose, crank_points)
        residual_norm = math.sqrt(sum(item * item for item in residuals))
        return {
            "reachable": residual_norm < 5.0,
            "converged": converged or residual_norm < 5.0,
            "residualNorm": residual_norm,
            "pose": pose.to_dict(),
            "base_points": self.base_points(),
            "crank_points": crank_points,
            "platform_points_world": platform_world,
            "motor_angles_deg": motor_angles_deg,
        }

    def solve_from_servo_angles(
        self,
        servo_angles_deg: list[float],
        initial_pose: Pose | None = None,
    ) -> dict:
        motor_angles_deg = self.servo_to_motor_angles(servo_angles_deg)
        solution = self.solve_pose_from_motor_angles(motor_angles_deg, initial_pose=initial_pose)
        solution["servo_angles_deg"] = servo_angles_deg
        return solution

    def solve(self, pose: Pose) -> dict:
        g = self.geometry
        base = self.base_points()
        platform_local = self.platform_points()
        rotation = self.rotation_matrix(pose)
        translation = [pose.x, pose.y, pose.z]

        platform_world: list[list[float]] = []
        crank_points: list[list[float]] = []
        motor_angles: list[float] = []
        servo_angles: list[float] = []
        pulses: list[int] = []
        reachable = True
        issues: list[str] = []

        for index in range(6):
            p_world = _mat_vec_mul(rotation, platform_local[index])
            q = [p_world[i] + translation[i] for i in range(3)]
            b = base[index]
            platform_world.append(q)

            l = sum((q[axis] - b[axis]) ** 2 for axis in range(3)) - (
                g.upper_leg**2 - g.lower_leg**2
            )
            m = 2 * g.lower_leg * (q[2] - b[2])
            plane = math.radians(g.stepper_plane_angles[index])
            n = 2 * g.lower_leg * (
                math.cos(plane) * (q[0] - b[0]) + math.sin(plane) * (q[1] - b[1])
            )
            denom = math.sqrt(m**2 + n**2)
            if denom == 0:
                reachable = False
                motor_angle = 0.0
                issues.append(f"M{index + 1}: singular denominator")
            else:
                ratio = l / denom
                if ratio < -1.0 or ratio > 1.0:
                    reachable = False
                    ratio = max(-1.0, min(1.0, ratio))
                    issues.append(f"M{index + 1}: workspace limit")
                motor_angle = math.degrees(math.asin(ratio)) - math.degrees(math.atan2(n, m))

            motor_angles.append(motor_angle)
            servo_angle = motor_angle * g.motor_signs[index] + g.zero_offsets_deg[index]
            servo_angles.append(servo_angle)
            pulses.append(round(servo_angle * g.servo_pulses_per_rev / 360.0))

            crank_points.append(
                [
                    g.lower_leg
                    * math.cos(math.radians(motor_angle))
                    * math.cos(plane)
                    + b[0],
                    g.lower_leg
                    * math.cos(math.radians(motor_angle))
                    * math.sin(plane)
                    + b[1],
                    g.lower_leg * math.sin(math.radians(motor_angle)) + b[2],
                ]
            )

        return {
            "reachable": reachable,
            "issues": issues,
            "pose": pose.to_dict(),
            "geometry": g.to_dict(),
            "base_points": base,
            "platform_points_local": platform_local,
            "platform_points_world": platform_world,
            "crank_points": crank_points,
            "motor_angles_deg": motor_angles,
            "servo_angles_deg": servo_angles,
            "motor_pulses": pulses,
        }
