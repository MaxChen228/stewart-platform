#include "forward_kinematics.h"
#include <math.h>

// 底座/平台附著點（與 kinematics.cpp 相同）
static void getBasePoint(int i, float out[3]) {
    float ang = BASE_ANGLES[i] * DEG2RAD;
    out[0] = BASE_RADIUS * cosf(ang);
    out[1] = BASE_RADIUS * sinf(ang);
    out[2] = 0;
}

static void getPlatformPoint(int i, float out[3]) {
    float ang = PLATFORM_ANGLES[i] * DEG2RAD;
    out[0] = PLATFORM_RADIUS * cosf(ang);
    out[1] = PLATFORM_RADIUS * sinf(ang);
    out[2] = 0;
}

// 從馬達角度計算膝蓋點 A（lower leg 頂端）
static void computeA(const float angles[6], float A[6][3]) {
    for (int i = 0; i < 6; i++) {
        float B[3];
        getBasePoint(i, B);
        float a = angles[i] * DEG2RAD;
        float th = MOTOR_PLANE_ANGLE[i] * DEG2RAD;
        A[i][0] = LOWER_LEG * cosf(a) * cosf(th) + B[0];
        A[i][1] = LOWER_LEG * cosf(a) * sinf(th) + B[1];
        A[i][2] = LOWER_LEG * sinf(a) + B[2];
    }
}

// 6×6 高斯消去（partial pivoting），回傳 false 表示奇異
static bool gaussSolve6(float J[6][7], float x[6]) {
    for (int i = 0; i < 6; i++) {
        // Partial pivoting
        int mx = i;
        for (int k = i + 1; k < 6; k++)
            if (fabsf(J[k][i]) > fabsf(J[mx][i])) mx = k;
        if (mx != i) {
            for (int j = 0; j < 7; j++) {
                float tmp = J[i][j]; J[i][j] = J[mx][j]; J[mx][j] = tmp;
            }
        }
        if (fabsf(J[i][i]) < 1e-10f) return false;

        // Forward elimination
        for (int k = i + 1; k < 6; k++) {
            float c = J[k][i] / J[i][i];
            for (int j = i; j < 7; j++) J[k][j] -= c * J[i][j];
        }
    }
    // Back substitution
    for (int i = 5; i >= 0; i--) {
        x[i] = J[i][6];
        for (int j = i + 1; j < 6; j++) x[i] -= J[i][j] * x[j];
        x[i] /= J[i][i];
    }
    return true;
}

Pose FKSolver::solve(const float angles[6]) {
    float A[6][3];
    computeA(angles, A);

    // Warm start: 上一幀的 pose
    float pose[6] = {lastPose.x, lastPose.y, lastPose.z,
                     lastPose.roll, lastPose.pitch, lastPose.yaw};

    // 安全檢查
    for (int i = 0; i < 6; i++) {
        if (!isfinite(pose[i])) { pose[0]=0; pose[1]=0; pose[2]=NEUTRAL_Z; pose[3]=0; pose[4]=0; pose[5]=0; break; }
    }

    float P[6][3]; // 平台附著點（platform frame）
    for (int i = 0; i < 6; i++) getPlatformPoint(i, P[i]);

    converged = false;
    iterations = 0;
    residual = 999;

    static constexpr float stepLimit[6] = {30, 30, 30, 15, 15, 15};
    static constexpr int MAX_ITER = 50;

    for (int iter = 0; iter < MAX_ITER; iter++) {
        float R[3][3], dRr[3][3], dRp[3][3], dRyw[3][3];
        rotation_matrix(pose[3], pose[4], pose[5], R);
        rotation_matrix_derivs(pose[3], pose[4], pose[5], dRr, dRp, dRyw);

        float f[6];
        float Aug[6][7]; // Augmented matrix [J | -f]

        float maxF = 0;
        for (int i = 0; i < 6; i++) {
            // Q = R * P[i] + T
            float qx = R[0][0]*P[i][0] + R[0][1]*P[i][1] + pose[0];
            float qy = R[1][0]*P[i][0] + R[1][1]*P[i][1] + pose[1];
            float qz = R[2][0]*P[i][0] + R[2][1]*P[i][1] + pose[2];

            float vx = qx - A[i][0];
            float vy = qy - A[i][1];
            float vz = qz - A[i][2];

            f[i] = vx*vx + vy*vy + vz*vz - UPPER_LEG*UPPER_LEG;

            // dR*P for roll, pitch, yaw
            float drx = dRr[0][0]*P[i][0] + dRr[0][1]*P[i][1];
            float dry = dRr[1][0]*P[i][0] + dRr[1][1]*P[i][1];
            float drz = dRr[2][0]*P[i][0] + dRr[2][1]*P[i][1];

            float dpx = dRp[0][0]*P[i][0] + dRp[0][1]*P[i][1];
            float dpy = dRp[1][0]*P[i][0] + dRp[1][1]*P[i][1];
            float dpz = dRp[2][0]*P[i][0] + dRp[2][1]*P[i][1];

            float dyx = dRyw[0][0]*P[i][0] + dRyw[0][1]*P[i][1];
            float dyy = dRyw[1][0]*P[i][0] + dRyw[1][1]*P[i][1];
            // dRyw[2] = [0,0,0], so dyz = 0

            // Jacobian row: [df/dx, df/dy, df/dz, df/droll, df/dpitch, df/dyaw]
            Aug[i][0] = 2*vx;
            Aug[i][1] = 2*vy;
            Aug[i][2] = 2*vz;
            Aug[i][3] = 2*(vx*drx + vy*dry + vz*drz) * DEG2RAD;
            Aug[i][4] = 2*(vx*dpx + vy*dpy + vz*dpz) * DEG2RAD;
            Aug[i][5] = 2*(vx*dyx + vy*dyy) * DEG2RAD;
            Aug[i][6] = -f[i]; // RHS

            if (fabsf(f[i]) > maxF) maxF = fabsf(f[i]);
        }

        iterations = iter;
        residual = maxF;

        if (maxF < 1.0f) {
            converged = true;
            break;
        }

        float dp[6];
        if (!gaussSolve6(Aug, dp)) break; // singular

        // Step limiting + update
        bool valid = true;
        for (int j = 0; j < 6; j++) {
            if (!isfinite(dp[j])) { valid = false; break; }
            if (dp[j] > stepLimit[j]) dp[j] = stepLimit[j];
            if (dp[j] < -stepLimit[j]) dp[j] = -stepLimit[j];
            pose[j] += dp[j];
        }
        if (!valid) break;
    }

    // 更新結果
    Pose result = {pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]};
    bool allFinite = true;
    for (int i = 0; i < 6; i++) { if (!isfinite(pose[i])) { allFinite = false; break; } }
    if (allFinite) lastPose = result;

    return converged ? result : lastPose;
}
