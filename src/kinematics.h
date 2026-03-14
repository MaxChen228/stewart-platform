#pragma once
#include <math.h>

// 幾何參數（mm, degrees）
constexpr float BASE_RADIUS = 152.0f;
constexpr float BASE_ANGLE = 18.92f;
constexpr float PLATFORM_RADIUS = 103.0f;
constexpr float PLATFORM_ANGLE = 28.07f;
constexpr float LOWER_LEG = 65.0f;
constexpr float UPPER_LEG = 165.0f;
constexpr float NEUTRAL_Z = 105.0f;

constexpr float DEG2RAD = M_PI / 180.0f;
constexpr float RAD2DEG = 180.0f / M_PI;

// 馬達平面角（度）: [-90, 90, 30, 210, -210, -30]
constexpr float MOTOR_PLANE_ANGLE[6] = {-90, 90, 30, 210, -210, -30};

// 底座附著點角度（度）
constexpr float BASE_ANGLES[6] = {
    -BASE_ANGLE / 2,
     BASE_ANGLE / 2,
    -BASE_ANGLE / 2 + 120,
     BASE_ANGLE / 2 + 120,
    -BASE_ANGLE / 2 - 120,
     BASE_ANGLE / 2 - 120
};

// 平台附著點角度（度）
constexpr float PLATFORM_ANGLES[6] = {
    -PLATFORM_ANGLE / 2,
     PLATFORM_ANGLE / 2,
    -PLATFORM_ANGLE / 2 + 120,
     PLATFORM_ANGLE / 2 + 120,
    -PLATFORM_ANGLE / 2 - 120,
     PLATFORM_ANGLE / 2 - 120
};

struct Pose {
    float x, y, z;
    float roll, pitch, yaw; // 度
};

struct IKResult {
    float angles[6]; // 馬達角度（度）
    bool valid;
};

// 逆向運動學：輸入姿態，輸出六個馬達角度
IKResult inverse_kinematics(const Pose& pose);
