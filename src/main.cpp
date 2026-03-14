#include <Arduino.h>
#include <Preferences.h>
#include "servo42d.h"
#include "kinematics.h"

Servo42D servos;
Preferences prefs;

int32_t zeroRaw[NUM_MOTORS] = {0};
float neutralAngles[NUM_MOTORS] = {0};
bool zeroed = false;

// ===== PID 控制 =====
Pose targetPose = {0, 0, NEUTRAL_Z, 0, 0, 0};
float pidKp = 3.0f;
float pidKi = 0.0f;
float pidKd = 1.0f;
float pidIntegral[NUM_MOTORS] = {0};
float pidPrevError[NUM_MOTORS] = {0};
float pidMaxRPM = 10.0f;
float pidMaxIntegral = 50.0f;
float pidMaxError = 20.0f; // 超過此角度誤差 → 急停（從 45 降到 20）
bool pidEnabled = false;

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

void pidReset() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        pidIntegral[i] = 0;
        pidPrevError[i] = 0;
    }
}

void pidStop() {
    pidEnabled = false;
    pidReset();
    servos.stopAllSpeed();
    servos.disableAll();
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
        pidStop();
        Serial.println("{\"status\":\"disabled all\"}");
    } else if (cmd == "E") {
        // 啟動 PID：先 enable 馬達再啟動控制
        pidReset();
        for (int i = 0; i < NUM_MOTORS; i++)
            servos.setEnable(MOTOR_ADDR[i], true);
        delay(10);
        pidEnabled = true;
        Serial.println("{\"status\":\"pid enabled\"}");
    } else if (cmd == "S") {
        // 停止 PID
        pidStop();
        Serial.println("{\"status\":\"pid stopped\"}");
    } else if (cmd.startsWith("P ")) {
        // 設定目標姿態: P x y z roll pitch yaw
        float v[6];
        if (sscanf(cmd.c_str(), "P %f %f %f %f %f %f", &v[0],&v[1],&v[2],&v[3],&v[4],&v[5]) == 6) {
            targetPose = {v[0], v[1], v[2], v[3], v[4], v[5]};
            Serial.printf("{\"status\":\"target set\",\"t\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]}\n",
                v[0],v[1],v[2],v[3],v[4],v[5]);
        }
    } else if (cmd.startsWith("K ")) {
        // 設定 PID 增益: K kp ki kd
        float kp, ki, kd;
        if (sscanf(cmd.c_str(), "K %f %f %f", &kp, &ki, &kd) == 3) {
            pidKp = kp; pidKi = ki; pidKd = kd;
            Serial.printf("{\"status\":\"pid gains\",\"kp\":%.1f,\"ki\":%.2f,\"kd\":%.1f}\n", kp, ki, kd);
        }
    } else if (cmd.startsWith("T")) {
        // 單馬達方向測試: T0~T5
        // 以 2 RPM 正轉 0.5 秒，回報角度變化方向
        int idx = cmd.charAt(1) - '0';
        if (idx >= 0 && idx < NUM_MOTORS) {
            servos.setEnable(MOTOR_ADDR[idx], true);
            delay(10);

            // 讀起始角度
            int32_t rawBefore = servos.readEncoderRaw(MOTOR_ADDR[idx]);

            // 以 dir=0 (CCW) 轉 0.5 秒
            servos.setSpeed(MOTOR_ADDR[idx], 3, 0, 0);
            delay(500);
            servos.setSpeed(MOTOR_ADDR[idx], 0, 0, 0);

            // 讀結束角度
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
    float dt = (millis() - lastRead) / 1000.0f;
    lastRead = millis();

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

    // PID 控制
    float errors[NUM_MOTORS] = {0};
    float rpms[NUM_MOTORS] = {0};

    if (pidEnabled) {
        IKResult target = inverse_kinematics(targetPose);
        if (!target.valid) {
            pidStop();
            Serial.println("{\"error\":\"IK invalid, PID stopped\"}");
        } else {
            bool safetyTrip = false;
            for (int i = 0; i < NUM_MOTORS; i++) {
                float error = target.angles[i] - angles[i];
                errors[i] = error;

                // 安全檢查
                if (fabsf(error) > pidMaxError) {
                    safetyTrip = true;
                    break;
                }

                // PID
                pidIntegral[i] += error * dt;
                pidIntegral[i] = constrain(pidIntegral[i], -pidMaxIntegral, pidMaxIntegral);
                float derivative = (error - pidPrevError[i]) / dt;
                pidPrevError[i] = error;

                float output = pidKp * error + pidKi * pidIntegral[i] + pidKd * derivative;
                // output = 角度速度 (deg/s)

                // 轉換為馬達軸速度（考慮 MOTOR_SIGN + speedDir）
                float shaftVel = output * MOTOR_SIGN[i] * speedDir[i];

                // 轉換為 RPM (360 deg/s = 60 RPM)
                float rpm = fabsf(shaftVel) / 6.0f;
                rpm = min(rpm, pidMaxRPM);
                rpms[i] = rpm;

                uint16_t speedCmd = 0;
                if (fabsf(error) > 1.5f) {
                    speedCmd = (uint16_t)roundf(rpm);
                    if (speedCmd == 0) speedCmd = 1;
                }
                uint8_t dir = shaftVel > 0 ? 1 : 0;
                servos.setSpeed(MOTOR_ADDR[i], speedCmd, dir, 0);
            }

            if (safetyTrip) {
                pidStop();
                Serial.println("{\"error\":\"angle error too large, PID stopped\"}");
            }
        }
    }

    // JSON 輸出
    Serial.printf("{\"a\":[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f],"
                  "\"r\":[%ld,%ld,%ld,%ld,%ld,%ld],"
                  "\"ok\":%d,\"z\":%d,\"pid\":%d",
        angles[0], angles[1], angles[2], angles[3], angles[4], angles[5],
        raw[0], raw[1], raw[2], raw[3], raw[4], raw[5],
        ok, zeroed ? 1 : 0, pidEnabled ? 1 : 0);

    if (pidEnabled) {
        Serial.printf(",\"err\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f],"
                      "\"rpm\":[%.0f,%.0f,%.0f,%.0f,%.0f,%.0f]",
            errors[0],errors[1],errors[2],errors[3],errors[4],errors[5],
            rpms[0],rpms[1],rpms[2],rpms[3],rpms[4],rpms[5]);
    }

    Serial.println("}");
}
