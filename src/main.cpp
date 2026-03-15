#include <Arduino.h>
#include <Preferences.h>
#include "servo42d.h"
#include "kinematics.h"

Servo42D servos;
Preferences prefs;

int32_t zeroRaw[NUM_MOTORS] = {0};
float neutralAngles[NUM_MOTORS] = {0};
bool zeroed = false;

// ===== 位置模式控制 =====
Pose targetPose = {0, 0, NEUTRAL_Z, 0, 0, 0};
Pose smoothedTarget = {0, 0, NEUTRAL_Z, 0, 0, 0};

bool posEnabled = false;
float motorZeroAngle[NUM_MOTORS] = {0}; // 啟動時各馬達的角度
int32_t coordBase[NUM_MOTORS] = {0};    // 啟動時馬達的 0x31 坐標值

uint16_t posSpeed = 30;   // 最大移動速度 (RPM)
uint8_t  posAcc   = 5;    // 加減速參數

constexpr float SMOOTH_TIME = 0.5f; // setpoint 平滑時間（秒）

// 逐馬達速度方向：1=正常, -1=翻轉
int8_t speedDir[NUM_MOTORS] = {1, 1, 1, 1, 1, 1};

void computeNeutralAngles() {
    Pose neutral = {0, 0, NEUTRAL_Z, 0, 0, 0};
    IKResult r = inverse_kinematics(neutral);
    for (int i = 0; i < NUM_MOTORS; i++) {
        neutralAngles[i] = r.angles[i];
    }
}

void saveZeroToNVS() {
    prefs.begin("stewart", false);
    prefs.putBytes("zeroRaw", zeroRaw, sizeof(zeroRaw));
    prefs.end();
}

bool loadZeroFromNVS() {
    prefs.begin("stewart", true);
    size_t len = prefs.getBytes("zeroRaw", zeroRaw, sizeof(zeroRaw));
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

// 角度 → 馬達絕對坐標值
// 基於 Enable 時讀取的 0x31 坐標 (coordBase) 和角度 (motorZeroAngle)
int32_t angleToCoord(int motorIdx, float targetAngle) {
    float deltaAngle = targetAngle - motorZeroAngle[motorIdx];
    int32_t deltaCoord = (int32_t)roundf(deltaAngle / (float)MOTOR_SIGN[motorIdx] * 16384.0f / 360.0f);
    return coordBase[motorIdx] + deltaCoord;
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
        // 設定馬達 F5 零點（0x92）— 在校正位置做是安全的
        for (int i = 0; i < NUM_MOTORS; i++) {
            servos.setEnable(MOTOR_ADDR[i], true);
            delay(5);
        }
        for (int i = 0; i < NUM_MOTORS; i++) {
            servos.setZeroPoint(MOTOR_ADDR[i]);
            delay(10);
        }
        servos.disableAll();

        // 0x92 之後重讀 encoder — 拿到的是 0x92 後的一致值
        int32_t raw[NUM_MOTORS];
        servos.readAllEncoders(raw);
        for (int i = 0; i < NUM_MOTORS; i++) zeroRaw[i] = raw[i];
        zeroed = true;
        saveZeroToNVS();
        Serial.println("{\"status\":\"zeroed all\",\"saved\":true}");
    } else if (cmd.startsWith("Z") && cmd.length() == 2) {
        int idx = cmd.charAt(1) - '0';
        if (idx >= 0 && idx < NUM_MOTORS) {
            int32_t val = servos.readEncoderRaw(MOTOR_ADDR[idx]);
            if (val >= 0) zeroRaw[idx] = val;
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

        // 1. 讀取當前角度（用 0x30，不受 0x92 影響）
        int32_t raw[NUM_MOTORS];
        servos.readAllEncoders(raw);
        for (int i = 0; i < NUM_MOTORS; i++) {
            int32_t d = raw[i] - zeroRaw[i];
            if (d > 8192) d -= 16384;
            if (d < -8192) d += 16384;
            motorZeroAngle[i] = MOTOR_SIGN[i] * (float)d * 360.0f / 16384.0f + 90.0f;
        }

        // 2. 讀取馬達當前坐標值（0x31）— F5 的坐標基準
        bool coordOk = true;
        for (int i = 0; i < NUM_MOTORS; i++) {
            int32_t c = servos.readCoordinate(MOTOR_ADDR[i]);
            if (c == INT32_MIN) {
                coordOk = false;
                break;
            }
            coordBase[i] = c;
        }
        if (!coordOk) {
            Serial.println("{\"error\":\"failed to read motor coordinates\"}");
            return;
        }

        // 3. 啟用馬達（不呼叫 0x92，避免破壞 0x30 讀數）
        for (int i = 0; i < NUM_MOTORS; i++)
            servos.setEnable(MOTOR_ADDR[i], true);
        delay(20);

        // 4. smoothedTarget 從當前 targetPose 開始
        smoothedTarget = targetPose;
        posEnabled = true;

        Serial.printf("{\"status\":\"pos enabled\",\"speed\":%d,\"acc\":%d,\"base\":[%ld,%ld,%ld,%ld,%ld,%ld]}\n",
            posSpeed, posAcc,
            coordBase[0],coordBase[1],coordBase[2],coordBase[3],coordBase[4],coordBase[5]);
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
        // 設定馬達內部 PID: K kp ki kd kv
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
        // 設定位置模式速度和加速度: V speed acc
        int spd, ac;
        if (sscanf(cmd.c_str(), "V %d %d", &spd, &ac) == 2) {
            posSpeed = constrain(spd, 1, 200);
            posAcc = constrain(ac, 1, 255);
            Serial.printf("{\"status\":\"pos params\",\"speed\":%d,\"acc\":%d}\n", posSpeed, posAcc);
        }
    } else if (cmd.startsWith("T")) {
        int idx = cmd.charAt(1) - '0';
        if (idx >= 0 && idx < NUM_MOTORS) {
            servos.setEnable(MOTOR_ADDR[idx], true);
            delay(10);

            int32_t rawBefore = servos.readEncoderRaw(MOTOR_ADDR[idx]);

            servos.setSpeed(MOTOR_ADDR[idx], 3, 0, 0);
            delay(500);
            servos.setSpeed(MOTOR_ADDR[idx], 0, 0, 0);

            int32_t rawAfter = servos.readEncoderRaw(MOTOR_ADDR[idx]);
            servos.setEnable(MOTOR_ADDR[idx], false);

            int32_t delta = rawAfter - rawBefore;
            if (delta > 8192) delta -= 16384;
            if (delta < -8192) delta += 16384;
            float degChange = MOTOR_SIGN[idx] * (float)delta * 360.0f / 16384.0f;

            Serial.printf("{\"status\":\"test M%d\",\"raw_delta\":%ld,\"deg_change\":%.2f,\"note\":\"%s\"}\n",
                idx + 1, delta, degChange,
                degChange > 0 ? "CCW=angle+" : "CCW=angle-");
        }
    }
}

void loop() {
    handleSerial();

    static uint32_t lastRead = 0;
    if (millis() - lastRead < 20) return;
    constexpr float FIXED_DT = 0.02f;
    uint32_t elapsed = millis() - lastRead;
    lastRead = millis();

    // 若實際間隔過大，跳過此 cycle
    if (elapsed > 50) return;

    int32_t raw[NUM_MOTORS];
    int ok = servos.readAllEncoders(raw);

    // 計算當前馬達角度
    float angles[NUM_MOTORS];
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (raw[i] < 0) {
            angles[i] = neutralAngles[i];
            continue;
        }
        int32_t d = raw[i] - zeroRaw[i];
        if (d > 8192) d -= 16384;
        if (d < -8192) d += 16384;
        float delta = (float)d * 360.0f / 16384.0f;
        angles[i] = MOTOR_SIGN[i] * delta + 90.0f;
    }

    // 位置模式控制
    float targetAngles[NUM_MOTORS] = {0};
    int32_t coords[NUM_MOTORS] = {0};

    if (posEnabled) {
        smoothPose(smoothedTarget, targetPose, FIXED_DT);

        IKResult target = inverse_kinematics(smoothedTarget);
        if (!target.valid) {
            posStop();
            Serial.println("{\"error\":\"IK invalid, pos stopped\"}");
        } else {
            for (int i = 0; i < NUM_MOTORS; i++) {
                targetAngles[i] = target.angles[i];
                coords[i] = angleToCoord(i, target.angles[i]);

                // 安全限制：坐標值不超過半圈
                if (abs(coords[i]) > 8192) {
                    posStop();
                    Serial.println("{\"error\":\"coord out of range, pos stopped\"}");
                    break;
                }

                servos.setAbsoluteCoord(MOTOR_ADDR[i], posSpeed, posAcc, coords[i]);
            }
        }
    }

    // JSON 輸出
    Serial.printf("{\"a\":[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f],"
                  "\"r\":[%ld,%ld,%ld,%ld,%ld,%ld],"
                  "\"ok\":%d,\"z\":%d,\"pos\":%d",
        angles[0], angles[1], angles[2], angles[3], angles[4], angles[5],
        raw[0], raw[1], raw[2], raw[3], raw[4], raw[5],
        ok, zeroed ? 1 : 0, posEnabled ? 1 : 0);

    if (posEnabled) {
        Serial.printf(",\"tgt\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f],"
                      "\"c\":[%ld,%ld,%ld,%ld,%ld,%ld]",
            targetAngles[0],targetAngles[1],targetAngles[2],
            targetAngles[3],targetAngles[4],targetAngles[5],
            coords[0],coords[1],coords[2],coords[3],coords[4],coords[5]);
    }

    Serial.println("}");
}
