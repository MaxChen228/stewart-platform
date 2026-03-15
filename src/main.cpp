#include <Arduino.h>
#include <Preferences.h>
#include "servo42d.h"
#include "kinematics.h"

Servo42D servos;
Preferences prefs;

int64_t zeroRaw[NUM_MOTORS] = {0};  // 0x35 raw encoder at Z calibration
float neutralAngles[NUM_MOTORS] = {0};
bool zeroed = false;
bool angleJumpGuardReset = false;   // 校正後重置跳變防護

// ===== 位置模式控制 =====
Pose targetPose = {0, 0, NEUTRAL_Z, 0, 0, 0};
Pose smoothedTarget = {0, 0, NEUTRAL_Z, 0, 0, 0};

bool posEnabled = false;
float enableAngle[NUM_MOTORS] = {0}; // Enable 時的角度（F5 coord 參考點，不影響映射）

uint16_t posSpeed = 30;   // 最大移動速度 (RPM)
uint8_t  posAcc   = 5;    // 加減速參數

constexpr float SMOOTH_TIME = 0.5f; // setpoint 平滑時間（秒）

// ===== 自適應追蹤 =====
float prevAngles[NUM_MOTORS] = {0};
float trackingMu = 0.5f;
float trackingKd = 3.0f;
float maxGain = 0.3f;              // 外迴圈最大增益（低=穩定，高=響應快但易震盪）
float smoothKinetic = 0;
bool adaptiveFirstCycle = true;

void computeNeutralAngles() {
    Pose neutral = {0, 0, NEUTRAL_Z, 0, 0, 0};
    IKResult r = inverse_kinematics(neutral);
    for (int i = 0; i < NUM_MOTORS; i++) {
        neutralAngles[i] = r.angles[i];
    }
}

void saveZeroToNVS() {
    prefs.begin("stewart", false);
    prefs.putBytes("zeroRaw64", zeroRaw, sizeof(zeroRaw));
    prefs.end();
}

bool loadZeroFromNVS() {
    prefs.begin("stewart", true);
    size_t len = prefs.getBytes("zeroRaw64", zeroRaw, sizeof(zeroRaw));
    prefs.end();
    return len == sizeof(zeroRaw);
}

// 停止追蹤但保持馬達使能（有保持力矩）
void posStop() {
    posEnabled = false;
    servos.stopAllPosition(5);
}

// 完全斷電，馬達自由轉動
void posDisable() {
    posEnabled = false;
    servos.stopAllPosition(5);
    delay(50);
    servos.disableAll();
}

static void smoothPose(Pose& sm, const Pose& tgt, float dt) {
    float alpha = fminf(1.0f, dt / SMOOTH_TIME);
    sm.x    += alpha * (tgt.x    - sm.x);
    sm.y    += alpha * (tgt.y    - sm.y);
    sm.z    += alpha * (tgt.z    - sm.z);
    sm.roll += alpha * (tgt.roll - sm.roll);
    sm.pitch+= alpha * (tgt.pitch- sm.pitch);
    sm.yaw  += alpha * (tgt.yaw  - sm.yaw);
}

// 角度 → F5 絕對座標（相對於 Enable 時的 0x92 零點）
// enableAngle 是 F5 的座標參考，不影響角度映射
int32_t angleToCoord(int motorIdx, float targetAngle) {
    float deltaAngle = targetAngle - enableAngle[motorIdx];
    return (int32_t)roundf(deltaAngle / (float)MOTOR_SIGN[motorIdx] * 16384.0f / 360.0f);
}

// 0x35 raw → 角度（純粹映射，只有 zeroRaw 決定）
float rawToAngle(int motorIdx, int64_t raw) {
    int64_t d = raw - zeroRaw[motorIdx];
    return MOTOR_SIGN[motorIdx] * (float)d * 360.0f / 16384.0f + 90.0f;
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    computeNeutralAngles();
    zeroed = loadZeroFromNVS();

    Serial.printf("{\"status\":\"init\",\"calibrated\":%s}\n",
                  zeroed ? "true" : "false");

    if (!servos.begin()) {
        Serial.println("{\"error\":\"CAN init failed\"}");
        while (1) delay(1000);
    }

    servos.disableAll();
    Serial.println("{\"status\":\"ready\"}");
}

void handleSerial() {
    if (!Serial.available()) return;
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "Z") {
        // 1. Enable + 0x92（設 F5 座標零點）
        for (int i = 0; i < NUM_MOTORS; i++) {
            servos.setEnable(MOTOR_ADDR[i], true);
            delay(5);
        }
        for (int i = 0; i < NUM_MOTORS; i++) {
            servos.setZeroPoint(MOTOR_ADDR[i]);
            delay(10);
        }
        servos.disableAll();
        delay(20);

        // 2. 清掉殘留回覆，用 0x35 讀 RAW encoder
        servos.flushReceiveBuffer();
        int64_t raw[NUM_MOTORS];
        int ok = servos.readAllRawEncoders(raw);

        // 3. 驗證全部讀取成功
        bool allOk = true;
        for (int i = 0; i < NUM_MOTORS; i++) {
            if (raw[i] == INT64_MIN) { allOk = false; break; }
        }
        if (!allOk) {
            Serial.printf("{\"error\":\"encoder read failed (%d/6 ok)\"}\n", ok);
            return;
        }

        for (int i = 0; i < NUM_MOTORS; i++) zeroRaw[i] = raw[i];
        zeroed = true;
        angleJumpGuardReset = true;
        saveZeroToNVS();
        Serial.println("{\"status\":\"zeroed all\",\"saved\":true}");
    } else if (cmd.startsWith("Z") && cmd.length() == 2) {
        int idx = cmd.charAt(1) - '0';
        if (idx >= 0 && idx < NUM_MOTORS) {
            int64_t val = servos.readRawEncoderValue(MOTOR_ADDR[idx]);
            if (val != INT64_MIN) {
                zeroRaw[idx] = val;
            } else {
                Serial.printf("{\"error\":\"M%d read failed\"}\n", idx + 1);
                return;
            }
            angleJumpGuardReset = true;
            saveZeroToNVS();
            Serial.printf("{\"status\":\"zeroed M%d\",\"saved\":true}\n", idx + 1);
        }
    } else if (cmd == "D") {
        posDisable();
        Serial.println("{\"status\":\"disabled all\"}");
    } else if (cmd == "E") {
        if (!zeroed) {
            Serial.println("{\"error\":\"not calibrated, run Z first\"}");
            return;
        }

        // 1. 啟用馬達 + 0x92（F5 座標零點，0x35 不受影響）
        for (int i = 0; i < NUM_MOTORS; i++)
            servos.setEnable(MOTOR_ADDR[i], true);
        delay(20);
        for (int i = 0; i < NUM_MOTORS; i++) {
            servos.setZeroPoint(MOTOR_ADDR[i]);
            delay(10);
        }
        servos.flushReceiveBuffer();

        // 2. 用 0x35 讀當前角度作為 F5 座標參考（0x35 不受 0x92 影響）
        int64_t raw[NUM_MOTORS];
        servos.readAllRawEncoders(raw);
        float initAngles[NUM_MOTORS];
        for (int i = 0; i < NUM_MOTORS; i++) {
            if (raw[i] != INT64_MIN) {
                initAngles[i] = rawToAngle(i, raw[i]);
            } else {
                initAngles[i] = 90.0f;
            }
            enableAngle[i] = initAngles[i]; // F5 coord 參考點
        }

        smoothedTarget = targetPose;
        for (int i = 0; i < NUM_MOTORS; i++)
            prevAngles[i] = initAngles[i];
        adaptiveFirstCycle = true;
        posEnabled = true;

        // Debug: 顯示 enableAngle 和 target 角度，找出「往下扯」的原因
        IKResult dbgIK = inverse_kinematics(targetPose);
        Serial.printf("{\"status\":\"pos enabled\",\"speed\":%d,\"acc\":%d,"
            "\"enableAngle\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f],"
            "\"ikTarget\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f],"
            "\"targetPose\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]}\n",
            posSpeed, posAcc,
            enableAngle[0],enableAngle[1],enableAngle[2],
            enableAngle[3],enableAngle[4],enableAngle[5],
            dbgIK.valid?dbgIK.angles[0]:0, dbgIK.valid?dbgIK.angles[1]:0,
            dbgIK.valid?dbgIK.angles[2]:0, dbgIK.valid?dbgIK.angles[3]:0,
            dbgIK.valid?dbgIK.angles[4]:0, dbgIK.valid?dbgIK.angles[5]:0,
            targetPose.x, targetPose.y, targetPose.z,
            targetPose.roll, targetPose.pitch, targetPose.yaw);
    } else if (cmd == "S") {
        posStop();
        Serial.println("{\"status\":\"pos stopped\"}");
    } else if (cmd.startsWith("P ")) {
        float v[6];
        if (sscanf(cmd.c_str(), "P %f %f %f %f %f %f", &v[0],&v[1],&v[2],&v[3],&v[4],&v[5]) == 6) {
            targetPose = {v[0], v[1], v[2], v[3], v[4], v[5]};
            Serial.printf("{\"status\":\"target set\",\"t\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]}\n",
                v[0],v[1],v[2],v[3],v[4],v[5]);
        }
    } else if (cmd.startsWith("K ")) {
        int kp, ki, kd, kv;
        if (sscanf(cmd.c_str(), "K %d %d %d %d", &kp, &ki, &kd, &kv) == 4) {
            for (int i = 0; i < NUM_MOTORS; i++) {
                servos.setVFOC_KpKi(MOTOR_ADDR[i], kp, ki);
                delay(5);
                servos.setVFOC_KdKv(MOTOR_ADDR[i], kd, kv);
                delay(5);
            }
            Serial.printf("{\"status\":\"motor PID set\",\"kp\":%d,\"ki\":%d,\"kd\":%d,\"kv\":%d}\n",
                kp, ki, kd, kv);
        }
    } else if (cmd.startsWith("V ")) {
        int spd, ac;
        if (sscanf(cmd.c_str(), "V %d %d", &spd, &ac) == 2) {
            posSpeed = constrain(spd, 1, 200);
            posAcc = constrain(ac, 1, 255);
            Serial.printf("{\"status\":\"pos params\",\"speed\":%d,\"acc\":%d}\n", posSpeed, posAcc);
        }
    } else if (cmd.startsWith("M ")) {
        float mu, kd = -1, mg = -1;
        int n = sscanf(cmd.c_str(), "M %f %f %f", &mu, &kd, &mg);
        if (n >= 1) {
            trackingMu = fmaxf(0.0f, mu);
            if (n >= 2) trackingKd = fmaxf(0.0f, kd);
            if (n >= 3) maxGain = fmaxf(0.05f, fminf(1.0f, mg));
            Serial.printf("{\"status\":\"tracking params\",\"mu\":%.2f,\"kd\":%.2f,\"maxGain\":%.2f}\n", trackingMu, trackingKd, maxGain);
        }
    } else if (cmd.startsWith("T")) {
        int idx = cmd.charAt(1) - '0';
        if (idx >= 0 && idx < NUM_MOTORS) {
            servos.setEnable(MOTOR_ADDR[idx], true);
            delay(10);

            int64_t rawBefore = servos.readRawEncoderValue(MOTOR_ADDR[idx]);

            servos.setSpeed(MOTOR_ADDR[idx], 3, 0, 0);
            delay(500);
            servos.setSpeed(MOTOR_ADDR[idx], 0, 0, 0);

            int64_t rawAfter = servos.readRawEncoderValue(MOTOR_ADDR[idx]);
            servos.setEnable(MOTOR_ADDR[idx], false);

            int64_t delta = rawAfter - rawBefore;
            float degChange = MOTOR_SIGN[idx] * (float)delta * 360.0f / 16384.0f;

            Serial.printf("{\"status\":\"test M%d\",\"raw_delta\":%lld,\"deg_change\":%.2f,\"note\":\"%s\"}\n",
                idx + 1, delta, degChange,
                degChange > 0 ? "CCW=angle+" : "CCW=angle-");
        }
    } else if (cmd == "I") {
        Serial.print("{\"diag\":{\"zeroRaw\":[");
        for (int i = 0; i < NUM_MOTORS; i++) Serial.printf("%s%lld", i?",":"", zeroRaw[i]);
        Serial.printf("],\"posEnabled\":%d}}\n", posEnabled ? 1 : 0);
    }
}

void loop() {
    handleSerial();

    static uint32_t lastRead = 0;
    if (millis() - lastRead < 20) return;
    constexpr float FIXED_DT = 0.02f;
    uint32_t elapsed = millis() - lastRead;
    lastRead = millis();

    if (elapsed > 50) return;

    servos.flushReceiveBuffer();

    // 用 0x35 讀 RAW encoder（不受 0x92 影響）
    int64_t raw[NUM_MOTORS];
    int ok = servos.readAllRawEncoders(raw);

    // 角度計算：angle = MOTOR_SIGN × (raw - zeroRaw) × 360/16384 + 90
    // zeroRaw 只有 Z 指令會改，其他任何操作都不會動到
    static float angles[NUM_MOTORS] = {0};
    static bool anglesInit = false;
    if (angleJumpGuardReset) {
        anglesInit = false;
        angleJumpGuardReset = false;
    }
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (raw[i] == INT64_MIN) continue; // 通訊失敗，保持上一次值

        float newAngle = rawToAngle(i, raw[i]);

        // 跳變防護：單 cycle >15° 視為異常
        if (anglesInit && fabsf(newAngle - angles[i]) > 15.0f) {
            Serial.printf("{\"warn\":\"M%d jump %.1f->%.1f, discarded\"}\n",
                          i+1, angles[i], newAngle);
            continue;
        }
        angles[i] = newAngle;
    }
    anglesInit = true;

    // 位置模式控制
    float targetAngles[NUM_MOTORS] = {0};
    float adjustedAngles[NUM_MOTORS] = {0};
    int32_t coords[NUM_MOTORS] = {0};
    float gain = 1.0f;

    if (posEnabled) {
        smoothPose(smoothedTarget, targetPose, FIXED_DT);

        IKResult target = inverse_kinematics(smoothedTarget);
        if (!target.valid) {
            posStop();
            Serial.println("{\"error\":\"IK invalid, pos stopped\"}");
        } else {
            if (adaptiveFirstCycle) {
                for (int i = 0; i < NUM_MOTORS; i++)
                    prevAngles[i] = angles[i];
                smoothKinetic = 0;
                adaptiveFirstCycle = false;
            }

            float vel[NUM_MOTORS];
            float kinetic = 0;
            for (int i = 0; i < NUM_MOTORS; i++) {
                vel[i] = angles[i] - prevAngles[i];
                kinetic += vel[i] * vel[i];
            }

            smoothKinetic = 0.9f * smoothKinetic + 0.1f * kinetic;
            gain = fminf(maxGain, 1.0f / (1.0f + trackingMu * smoothKinetic));

            for (int i = 0; i < NUM_MOTORS; i++) {
                targetAngles[i] = target.angles[i];

                float error = target.angles[i] - angles[i];
                // 非對稱阻尼：朝目標移動 30% 阻尼（防止蓄積動能），過衝 100% 煞車
                float dampRatio = (error * vel[i] < 0) ? 1.0f : 0.3f;
                float damp = trackingKd * vel[i] * dampRatio;
                adjustedAngles[i] = angles[i] + gain * error - damp;

                coords[i] = angleToCoord(i, adjustedAngles[i]);

                if (abs(coords[i]) > 8192) {
                    posStop();
                    Serial.println("{\"error\":\"coord out of range, pos stopped\"}");
                    break;
                }

                servos.setAbsoluteCoord(MOTOR_ADDR[i], posSpeed, posAcc, coords[i]);

                prevAngles[i] = angles[i];
            }
        }
    }

    // JSON 輸出
    Serial.printf("{\"a\":[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f],"
                  "\"r\":[%lld,%lld,%lld,%lld,%lld,%lld],"
                  "\"ok\":%d,\"z\":%d,\"pos\":%d",
        angles[0], angles[1], angles[2], angles[3], angles[4], angles[5],
        raw[0], raw[1], raw[2], raw[3], raw[4], raw[5],
        ok, zeroed ? 1 : 0, posEnabled ? 1 : 0);

    if (posEnabled) {
        Serial.printf(",\"tgt\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f],"
                      "\"adj\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f],"
                      "\"g\":%.3f,\"kd\":%.1f",
            targetAngles[0],targetAngles[1],targetAngles[2],
            targetAngles[3],targetAngles[4],targetAngles[5],
            adjustedAngles[0],adjustedAngles[1],adjustedAngles[2],
            adjustedAngles[3],adjustedAngles[4],adjustedAngles[5],
            gain, trackingKd);
    }

    Serial.println("}");
}
