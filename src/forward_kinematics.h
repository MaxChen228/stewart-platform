#pragma once
#include "kinematics.h"

struct FKSolver {
    Pose lastPose;
    int iterations = 0;
    float residual = 999;
    bool converged = false;

    FKSolver() : lastPose{0, 0, NEUTRAL_Z, 0, 0, 0} {}

    // 從 6 個馬達角度求解平台姿態（Newton-Raphson）
    Pose solve(const float angles[6]);

    // 重置 warm start 到中性姿態
    void reset() { lastPose = {0, 0, NEUTRAL_Z, 0, 0, 0}; }
};
