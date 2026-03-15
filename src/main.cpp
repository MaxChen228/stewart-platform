#include <Arduino.h>
#include "servo42d.h"
#include "kinematics.h"
#include "encoder.h"
#include "forward_kinematics.h"
#include "control.h"

Servo42D servos;
EncoderState enc;

// ===== 位置模式控制 =====
Pose targetPose = {0, 0, NEUTRAL_Z, 0, 0, 0};
bool posEnabled = false;

uint16_t posSpeed = 30;
uint8_t  posAcc   = 5;
float trajDuration = 1.0f;  // 軌跡持續時間（秒）

// ===== 控制器 =====
TrajectoryController controller;
bool syncEnabled = false;

void posStop() {
    posEnabled = false;
    servos.stopAllPosition(5);
}

void posDisable() {
    posEnabled = false;
    servos.stopAllPosition(5);
    delay(50);
    servos.enableSync(false);
    syncEnabled = false;
    servos.disableAll();
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    enc.init();

    Serial.printf("{\"status\":\"init\",\"calibrated\":%s}\n",
                  enc.zeroed ? "true" : "false");

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
        if (enc.zeroAll(servos)) {
            Serial.println("{\"status\":\"zeroed all\",\"saved\":true}");
        } else {
            Serial.println("{\"error\":\"encoder read failed during zeroing\"}");
        }
    } else if (cmd.startsWith("Z") && cmd.length() == 2) {
        int idx = cmd.charAt(1) - '0';
        if (idx >= 0 && idx < NUM_MOTORS) {
            if (enc.zeroMotor(idx, servos)) {
                Serial.printf("{\"status\":\"zeroed M%d\",\"saved\":true}\n", idx + 1);
            } else {
                Serial.printf("{\"error\":\"M%d read failed\"}\n", idx + 1);
            }
        }
    } else if (cmd == "D") {
        posDisable();
        Serial.println("{\"status\":\"disabled all\"}");
    } else if (cmd == "E") {
        if (!enc.zeroed) {
            Serial.println("{\"error\":\"not calibrated, run Z first\"}");
            return;
        }

        // 啟用馬達 + 0x92（F5/F4 座標零點）
        for (int i = 0; i < NUM_MOTORS; i++)
            servos.setEnable(MOTOR_ADDR[i], true);
        delay(20);
        for (int i = 0; i < NUM_MOTORS; i++) {
            servos.setZeroPoint(MOTOR_ADDR[i]);
            delay(10);
        }
        servos.flushReceiveBuffer();

        // 啟用同步模式
        servos.enableSync(true);
        syncEnabled = true;
        delay(10);

        // 初始化控制器
        controller.reset(enc.angles);
        // 開始軌跡：從當前位置到 targetPose
        controller.startTrajectory(enc.angles, targetPose, trajDuration);

        posEnabled = true;
        Serial.printf("{\"status\":\"pos enabled\",\"speed\":%d,\"acc\":%d,\"dur\":%.1f}\n",
            posSpeed, posAcc, trajDuration);
    } else if (cmd == "S") {
        posStop();
        if (syncEnabled) {
            servos.enableSync(false);
            syncEnabled = false;
        }
        Serial.println("{\"status\":\"pos stopped\"}");
    } else if (cmd.startsWith("P ")) {
        float v[6];
        if (sscanf(cmd.c_str(), "P %f %f %f %f %f %f", &v[0],&v[1],&v[2],&v[3],&v[4],&v[5]) == 6) {
            targetPose = {v[0], v[1], v[2], v[3], v[4], v[5]};
            // 若正在控制中，啟動新軌跡
            if (posEnabled) {
                controller.startTrajectory(enc.angles, targetPose, trajDuration);
            }
            Serial.printf("{\"status\":\"target set\",\"t\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]}\n",
                v[0],v[1],v[2],v[3],v[4],v[5]);
        }
    } else if (cmd.startsWith("C ")) {
        // 控制參數：C kp [duration]
        float kp = -1, dur = -1;
        int n = sscanf(cmd.c_str(), "C %f %f", &kp, &dur);
        if (n >= 1) {
            controller.Kp = fmaxf(0.0f, fminf(1.0f, kp));
            if (n >= 2) trajDuration = fmaxf(0.1f, dur);
            Serial.printf("{\"status\":\"control params\",\"kp\":%.3f,\"dur\":%.1f}\n",
                controller.Kp, trajDuration);
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
            Serial.printf("{\"status\":\"test M%d\",\"raw_delta\":%lld,\"deg_change\":%.2f}\n",
                idx + 1, delta, degChange);
        }
    } else if (cmd.startsWith("R ")) {
        float v[6];
        if (sscanf(cmd.c_str(), "R %f %f %f %f %f %f", &v[0],&v[1],&v[2],&v[3],&v[4],&v[5]) == 6) {
            Pose testPose = {v[0], v[1], v[2], v[3], v[4], v[5]};
            IKResult ik = inverse_kinematics(testPose);
            if (!ik.valid) {
                Serial.println("{\"error\":\"IK invalid for test pose\"}");
            } else {
                FKSolver fk;
                Pose r = fk.solve(ik.angles);
                Serial.printf("{\"roundtrip\":{\"err\":[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f],\"iter\":%d}}\n",
                    r.x-v[0], r.y-v[1], r.z-v[2], r.roll-v[3], r.pitch-v[4], r.yaw-v[5],
                    fk.iterations);
            }
        }
    } else if (cmd == "F") {
        FKSolver fk;
        Pose p = fk.solve(enc.angles);
        Serial.printf("{\"fk\":[%.2f,%.2f,%.2f,%.3f,%.3f,%.3f],\"iter\":%d,\"err\":%.3f,\"ok\":%d}\n",
            p.x, p.y, p.z, p.roll, p.pitch, p.yaw,
            fk.iterations, fk.residual, fk.converged ? 1 : 0);
    } else if (cmd == "I") {
        Serial.print("{\"diag\":{\"zeroRaw\":[");
        for (int i = 0; i < NUM_MOTORS; i++) Serial.printf("%s%lld", i?",":"", enc.zeroRaw[i]);
        Serial.printf("],\"posEnabled\":%d,\"sync\":%d}}\n", posEnabled ? 1 : 0, syncEnabled ? 1 : 0);
    }
}

void loop() {
    handleSerial();

    static uint32_t lastRead = 0;
    if (millis() - lastRead < 20) return;
    uint32_t elapsed = millis() - lastRead;
    lastRead = millis();
    if (elapsed > 50) return;

    constexpr float DT = 0.02f;

    servos.flushReceiveBuffer();

    int64_t raw[NUM_MOTORS];
    int ok = servos.readAllRawEncoders(raw);
    enc.updateAngles(raw);

    if (posEnabled) {
        int32_t increments[NUM_MOTORS] = {0};
        bool controlOk = controller.update(enc.angles, DT, increments);

        if (controller.fkFailCount >= 5) {
            posStop();
            Serial.println("{\"error\":\"FK fail x5, stopped\"}");
        }

        if (controlOk) {
            // 安全限制：單 cycle 增量不超過 ±500 counts (≈11°)
            bool safe = true;
            for (int i = 0; i < NUM_MOTORS; i++) {
                if (abs(increments[i]) > 500) {
                    posStop();
                    Serial.printf("{\"error\":\"increment too large M%d=%d, stopped\"}\n",
                        i + 1, increments[i]);
                    safe = false;
                    break;
                }
            }

            if (safe) {
                // F4 相對座標 → 各馬達（同步模式下緩衝）
                for (int i = 0; i < NUM_MOTORS; i++) {
                    servos.setRelativeCoord(MOTOR_ADDR[i], posSpeed, posAcc, increments[i]);
                }
                // 觸發同步執行
                servos.triggerSync();
                delayMicroseconds(500);
                servos.triggerSync(); // 重複確保收到
            }
        }
    }

    // JSON 輸出
    Serial.printf("{\"a\":[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f],"
                  "\"r\":[%lld,%lld,%lld,%lld,%lld,%lld],"
                  "\"ok\":%d,\"z\":%d,\"pos\":%d",
        enc.angles[0], enc.angles[1], enc.angles[2],
        enc.angles[3], enc.angles[4], enc.angles[5],
        raw[0], raw[1], raw[2], raw[3], raw[4], raw[5],
        ok, enc.zeroed ? 1 : 0, posEnabled ? 1 : 0);

    if (posEnabled) {
        const Pose& fp = controller.actualPose;
        const Pose& te = controller.trackingError;
        Serial.printf(",\"fk\":[%.1f,%.1f,%.1f,%.2f,%.2f,%.2f],"
                      "\"err\":[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f],"
                      "\"prog\":%.2f,\"fki\":%d",
            fp.x, fp.y, fp.z, fp.roll, fp.pitch, fp.yaw,
            te.x, te.y, te.z, te.roll, te.pitch, te.yaw,
            controller.traj.progress(), controller.fk.iterations);
    }

    Serial.println("}");
}
