#pragma once
#include "kinematics.h"
#include "forward_kinematics.h"

// 單軸純量積分器（gain-independent）。state 存 ∫err·dt（與增益無關），
// 輸出時才乘 Ki 並做角度/pose 域硬 clamp + back-calculation anti-windup。
// 設計理由：調 Ki 不改寫積分歷史、不產生 retune 跳變；clamp 是最終安全閥。
// HOLD（角度域 6 份）與 mode 1 task-space（pose 域 6 份）共用此單一邏輯，零重複。
struct ScalarIntegrator {
    float accum = 0.0f;   // ∫err·dt

    void reset() { accum = 0.0f; }

    // gated：呼叫端判定的「可積分」條件（安定閘 / 死區 / stale 都在外面判）。
    // ki<=0 視為積分關閉：不累積、不輸出、不碰 accum（向後相容：BOOT Ki=0 即 no-op）。
    // 回傳本軸積分輸出 corr = clamp(ki*accum, ±clamp)；飽和時回拉 accum 防 windup。
    float update(float err, float dt, float ki, float clamp, bool gated) {
        if (ki <= 0.0f) return 0.0f;
        if (gated && dt > 0.0f) accum += err * dt;
        float out = ki * accum;
        if (out > clamp)       { out = clamp;  accum = out / ki; }
        else if (out < -clamp) { out = -clamp; accum = out / ki; }
        return out;
    }
};

struct TaskSpacePD {
    // 增益：[x, y, z, roll, pitch, yaw]
    // Kp = 每 cycle 修正的誤差比例（0~1）
    // Kd = 阻尼係數（秒），Kd/dt 為速度懲罰比例
    float Kp[6] = {0.15f, 0.15f, 0.15f, 0.15f, 0.15f, 0.15f};
    float Kd[6] = {0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f};
    // mode1 pose 域積分（消 task-space 穩態誤差）。Ki 預設 0 = no-op（向後相容）。
    // 與 HOLD 角度域共用 ScalarIntegrator；clamp 分位置(mm)/旋轉(deg)兩域，量級不同故不可共用單值。
    float Ki[6] = {0, 0, 0, 0, 0, 0};
    float iClampPos = 0.5f;   // 位置軸積分輸出上限 (mm)
    float iClampRot = 2.0f;   // 旋轉軸積分輸出上限 (deg)
    ScalarIntegrator integ[6];

    FKSolver fk;
    Pose prevPose;
    bool firstCycle = true;
    int fkFailCount = 0;

    // 遙測（供 JSON 輸出）
    Pose currentPose, poseError, controlPose;

    TaskSpacePD() : prevPose{0, 0, NEUTRAL_Z, 0, 0, 0} {}

    void reset(const float angles[6]) {
        fk.reset();
        currentPose = fk.solve(angles);
        prevPose = currentPose;
        firstCycle = true;
        fkFailCount = 0;
        for (int i = 0; i < 6; i++) integ[i].reset();   // 重入(E enable / fallback)清積分態
    }

    // 核心：從當前角度 + 目標姿態 → 輸出馬達目標角度
    // 回傳 false = FK 或 IK 失敗
    // dt = 本 cycle 真實取樣間隔（秒）。D 項 vel=Δpose/dt 必須用真實間隔，
    // 否則改 L 指令（迴圈週期）時 D 項與 Kd 語義會錯誤縮放（見 audit #1）。
    bool update(const float currentAngles[6], const Pose& target,
                float outAngles[6], float dt) {
        const float DT = dt;

        // 1. FK：從馬達角度恢復平台姿態
        currentPose = fk.solve(currentAngles);
        if (!fk.converged) {
            fkFailCount++;
            return false;
        }
        fkFailCount = 0;

        // 2. Task-space 誤差
        float err[6] = {
            target.x    - currentPose.x,
            target.y    - currentPose.y,
            target.z    - currentPose.z,
            target.roll - currentPose.roll,
            target.pitch- currentPose.pitch,
            target.yaw  - currentPose.yaw
        };
        poseError = {err[0], err[1], err[2], err[3], err[4], err[5]};

        // 3. Task-space 速度（有限差分）
        float vel[6] = {0};
        if (!firstCycle) {
            vel[0] = (currentPose.x     - prevPose.x)     / DT;
            vel[1] = (currentPose.y     - prevPose.y)     / DT;
            vel[2] = (currentPose.z     - prevPose.z)     / DT;
            vel[3] = (currentPose.roll  - prevPose.roll)  / DT;
            vel[4] = (currentPose.pitch - prevPose.pitch) / DT;
            vel[5] = (currentPose.yaw   - prevPose.yaw)   / DT;
        }

        // 4. PD 控制：controlPose = currentPose + Kp·error - Kd·velocity
        float cp[6] = {
            currentPose.x,     currentPose.y,     currentPose.z,
            currentPose.roll,  currentPose.pitch,  currentPose.yaw
        };
        for (int i = 0; i < 6; i++) {
            float iClamp = (i < 3) ? iClampPos : iClampRot;   // 位置(mm)/旋轉(deg)分域選 scalar
            cp[i] += Kp[i] * err[i] - Kd[i] * vel[i]
                   + integ[i].update(err[i], DT, Ki[i], iClamp, !firstCycle);  // gated=!firstCycle
        }
        controlPose = {cp[0], cp[1], cp[2], cp[3], cp[4], cp[5]};

        // 5. IK：controlPose → 馬達目標角度
        IKResult ik = inverse_kinematics(controlPose);
        if (!ik.valid) {
            // IK 無解 → 嘗試折半（currentPose 和 target 的中點）
            Pose mid = {
                (currentPose.x + target.x) * 0.5f,
                (currentPose.y + target.y) * 0.5f,
                (currentPose.z + target.z) * 0.5f,
                (currentPose.roll + target.roll) * 0.5f,
                (currentPose.pitch + target.pitch) * 0.5f,
                (currentPose.yaw + target.yaw) * 0.5f
            };
            ik = inverse_kinematics(mid);
            if (!ik.valid) {
                prevPose = currentPose;
                firstCycle = false;
                return false;
            }
            controlPose = mid;
        }

        for (int i = 0; i < 6; i++) outAngles[i] = ik.angles[i];

        prevPose = currentPose;
        firstCycle = false;
        return true;
    }
};
