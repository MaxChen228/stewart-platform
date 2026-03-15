#pragma once
#include "kinematics.h"

struct Trajectory {
    Pose startPose, endPose;
    float duration = 1.0f;    // 軌跡持續時間（秒）
    float elapsed = 0;
    bool active = false;

    // 五次多項式 smoothstep：s(0)=0, s(1)=1, s'(0)=s'(1)=0, s''(0)=s''(1)=0
    static float quintic(float t) {
        if (t <= 0) return 0;
        if (t >= 1) return 1;
        return t * t * t * (10.0f - 15.0f * t + 6.0f * t * t);
    }

    // 開始新軌跡
    void start(const Pose& from, const Pose& to, float dur) {
        startPose = from;
        endPose = to;
        duration = fmaxf(0.1f, dur);
        elapsed = 0;
        active = true;
    }

    // 更新目標（不重置起點，用於滑桿拖動時平滑跟隨）
    void retarget(const Pose& to) {
        endPose = to;
    }

    // 每 cycle 推進 dt 秒，回傳當前插值姿態
    Pose advance(float dt) {
        if (!active) return endPose;

        elapsed += dt;
        float t = elapsed / duration;
        float s = quintic(t);

        Pose p;
        p.x     = startPose.x     + s * (endPose.x     - startPose.x);
        p.y     = startPose.y     + s * (endPose.y     - startPose.y);
        p.z     = startPose.z     + s * (endPose.z     - startPose.z);
        p.roll  = startPose.roll  + s * (endPose.roll  - startPose.roll);
        p.pitch = startPose.pitch + s * (endPose.pitch - startPose.pitch);
        p.yaw   = startPose.yaw   + s * (endPose.yaw   - startPose.yaw);

        if (t >= 1.0f) active = false;
        return p;
    }

    float progress() const {
        return active ? fminf(1.0f, elapsed / duration) : 1.0f;
    }
};
