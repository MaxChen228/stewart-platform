#include "kinematics.h"

// 底座附著點（預計算，在 base frame）
static void get_base_point(int i, float out[3]) {
    float ang = BASE_ANGLES[i] * DEG2RAD;
    out[0] = BASE_RADIUS * cosf(ang);
    out[1] = BASE_RADIUS * sinf(ang);
    out[2] = 0;
}

// 平台附著點（在 platform frame，未旋轉平移）
static void get_platform_point(int i, float out[3]) {
    float ang = PLATFORM_ANGLES[i] * DEG2RAD;
    out[0] = PLATFORM_RADIUS * cosf(ang);
    out[1] = PLATFORM_RADIUS * sinf(ang);
    out[2] = 0;
}

// R = Rz(yaw) * Ry(pitch) * Rx(roll)，與參考專案一致
void rotation_matrix(float roll_deg, float pitch_deg, float yaw_deg, float R[3][3]) {
    float cr = cosf(roll_deg * DEG2RAD),  sr = sinf(roll_deg * DEG2RAD);
    float cp = cosf(pitch_deg * DEG2RAD), sp = sinf(pitch_deg * DEG2RAD);
    float cy = cosf(yaw_deg * DEG2RAD),   sy = sinf(yaw_deg * DEG2RAD);

    R[0][0] = cy * cp;
    R[0][1] = cy * sp * sr - sy * cr;
    R[0][2] = cy * sp * cr + sy * sr;
    R[1][0] = sy * cp;
    R[1][1] = sy * sp * sr + cy * cr;
    R[1][2] = sy * sp * cr - cy * sr;
    R[2][0] = -sp;
    R[2][1] = cp * sr;
    R[2][2] = cp * cr;
}

IKResult inverse_kinematics(const Pose& pose) {
    IKResult result;
    result.valid = true;

    float R[3][3];
    rotation_matrix(pose.roll, pose.pitch, pose.yaw, R);

    for (int i = 0; i < 6; i++) {
        float B[3], P[3], Q[3];
        get_base_point(i, B);
        get_platform_point(i, P);

        // Q = R * P + T
        Q[0] = R[0][0] * P[0] + R[0][1] * P[1] + R[0][2] * P[2] + pose.x;
        Q[1] = R[1][0] * P[0] + R[1][1] * P[1] + R[1][2] * P[2] + pose.y;
        Q[2] = R[2][0] * P[0] + R[2][1] * P[1] + R[2][2] * P[2] + pose.z;

        float dx = Q[0] - B[0];
        float dy = Q[1] - B[1];
        float dz = Q[2] - B[2];

        float L = dx * dx + dy * dy + dz * dz
                  - (UPPER_LEG * UPPER_LEG - LOWER_LEG * LOWER_LEG);
        float M = 2.0f * LOWER_LEG * dz;
        float theta = MOTOR_PLANE_ANGLE[i] * DEG2RAD;
        float N = 2.0f * LOWER_LEG * (cosf(theta) * dx + sinf(theta) * dy);

        float denom = sqrtf(M * M + N * N);
        float sin_arg = L / denom;

        if (fabsf(sin_arg) > 1.0f) {
            result.valid = false;
            result.angles[i] = 0;
            continue;
        }

        result.angles[i] = (asinf(sin_arg) - atan2f(N, M)) * RAD2DEG;
    }

    return result;
}

void rotation_matrix_derivs(float roll_deg, float pitch_deg, float yaw_deg,
                             float dRr[3][3], float dRp[3][3], float dRyw[3][3]) {
    float cr = cosf(roll_deg * DEG2RAD),  sr = sinf(roll_deg * DEG2RAD);
    float cp = cosf(pitch_deg * DEG2RAD), sp = sinf(pitch_deg * DEG2RAD);
    float cy = cosf(yaw_deg * DEG2RAD),   sy = sinf(yaw_deg * DEG2RAD);

    // dR/droll
    dRr[0][0] = 0;           dRr[0][1] = cy*sp*cr+sy*sr;   dRr[0][2] = -cy*sp*sr+sy*cr;
    dRr[1][0] = 0;           dRr[1][1] = sy*sp*cr-cy*sr;   dRr[1][2] = -sy*sp*sr-cy*cr;
    dRr[2][0] = 0;           dRr[2][1] = cp*cr;            dRr[2][2] = -cp*sr;

    // dR/dpitch
    dRp[0][0] = -cy*sp;      dRp[0][1] = cy*cp*sr;         dRp[0][2] = cy*cp*cr;
    dRp[1][0] = -sy*sp;      dRp[1][1] = sy*cp*sr;         dRp[1][2] = sy*cp*cr;
    dRp[2][0] = -cp;         dRp[2][1] = -sp*sr;           dRp[2][2] = -sp*cr;

    // dR/dyaw
    dRyw[0][0] = -sy*cp;     dRyw[0][1] = -sy*sp*sr-cy*cr; dRyw[0][2] = -sy*sp*cr+cy*sr;
    dRyw[1][0] = cy*cp;      dRyw[1][1] = cy*sp*sr-sy*cr;  dRyw[1][2] = cy*sp*cr+sy*sr;
    dRyw[2][0] = 0;          dRyw[2][1] = 0;               dRyw[2][2] = 0;
}
