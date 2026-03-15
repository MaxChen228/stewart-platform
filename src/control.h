#pragma once
#include "kinematics.h"
#include "forward_kinematics.h"
#include "trajectory.h"

struct TrajectoryController {
    // 增益（task-space 回饋修正用，小值即可）
    float Kp = 0.1f;

    FKSolver fk;
    Trajectory traj;
    int fkFailCount = 0;

    // 遙測
    Pose plannedPose;     // 軌跡規劃的當前點
    Pose actualPose;      // FK 估算的實際姿態
    Pose trackingError;   // planned - actual

    TrajectoryController() {}

    void reset(const float angles[6]) {
        fk.reset();
        actualPose = fk.solve(angles);
        fkFailCount = 0;
    }

    // 開始新軌跡（從當前 FK 姿態到 target，持續 duration 秒）
    void startTrajectory(const float angles[6], const Pose& target, float duration) {
        actualPose = fk.solve(angles);
        traj.start(actualPose, target, duration);
    }

    // 核心：每 cycle 呼叫
    // 輸出：outIncrements[6] = 每個馬達的 encoder count 增量（用於 F4）
    // 回傳 false = FK 或 IK 失敗
    bool update(const float currentAngles[6], float dt, int32_t outIncrements[6]) {
        // 1. FK：估算實際姿態
        actualPose = fk.solve(currentAngles);
        if (!fk.converged) { fkFailCount++; return false; }
        fkFailCount = 0;

        // 2. 軌跡推進
        plannedPose = traj.advance(dt);

        // 3. Task-space 追蹤誤差
        trackingError = {
            plannedPose.x     - actualPose.x,
            plannedPose.y     - actualPose.y,
            plannedPose.z     - actualPose.z,
            plannedPose.roll  - actualPose.roll,
            plannedPose.pitch - actualPose.pitch,
            plannedPose.yaw   - actualPose.yaw
        };

        // 4. 修正後的目標姿態 = planned + 小量回饋修正
        Pose corrected = {
            plannedPose.x     + Kp * trackingError.x,
            plannedPose.y     + Kp * trackingError.y,
            plannedPose.z     + Kp * trackingError.z,
            plannedPose.roll  + Kp * trackingError.roll,
            plannedPose.pitch + Kp * trackingError.pitch,
            plannedPose.yaw   + Kp * trackingError.yaw
        };

        // 5. IK → 目標馬達角度
        IKResult ik = inverse_kinematics(corrected);
        if (!ik.valid) {
            // 回退：用 plannedPose 不加修正
            ik = inverse_kinematics(plannedPose);
            if (!ik.valid) return false;
        }

        // 6. 計算 F4 增量 = (目標角度 - 實際角度) 轉 encoder counts
        for (int i = 0; i < 6; i++) {
            float deltaAngle = ik.angles[i] - currentAngles[i];
            outIncrements[i] = (int32_t)roundf(
                deltaAngle / (float)MOTOR_SIGN[i] * 16384.0f / 360.0f);
        }

        return true;
    }
};
