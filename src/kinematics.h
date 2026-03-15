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

// 馬達平面角（度）— CW 排列：pair@210°, pair@90°, pair@-30°
// 實體俯視：M1,M2 左下 / M3,M4 上方 / M5,M6 右側
// CW 走向：奇數馬達在 pair_center+90°，偶數在 pair_center-90°
constexpr float MOTOR_PLANE_ANGLE[6] = {300, 120, 180, 0, 60, 240};

// 底座附著點角度（度）— CW
// 奇數馬達在 pair_center + BASE_ANGLE/2，偶數在 pair_center - BASE_ANGLE/2
constexpr float BASE_ANGLES[6] = {
     BASE_ANGLE / 2 + 210,
    -BASE_ANGLE / 2 + 210,
     BASE_ANGLE / 2 + 90,
    -BASE_ANGLE / 2 + 90,
     BASE_ANGLE / 2 - 30,
    -BASE_ANGLE / 2 - 30
};

// 平台附著點角度（度）— CW
constexpr float PLATFORM_ANGLES[6] = {
     PLATFORM_ANGLE / 2 + 210,
    -PLATFORM_ANGLE / 2 + 210,
     PLATFORM_ANGLE / 2 + 90,
    -PLATFORM_ANGLE / 2 + 90,
     PLATFORM_ANGLE / 2 - 30,
    -PLATFORM_ANGLE / 2 - 30
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

// 旋轉矩陣 R = Rz(yaw) * Ry(pitch) * Rx(roll)（IK/FK 共用）
void rotation_matrix(float roll_deg, float pitch_deg, float yaw_deg, float R[3][3]);

// 旋轉矩陣偏導數（FK Jacobian 用）
void rotation_matrix_derivs(float roll_deg, float pitch_deg, float yaw_deg,
                             float dRr[3][3], float dRp[3][3], float dRyw[3][3]);
