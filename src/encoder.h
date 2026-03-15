#pragma once
#include <Preferences.h>
#include "servo42d.h"
#include "kinematics.h"

struct EncoderState {
    int64_t zeroRaw[NUM_MOTORS] = {0};
    float angles[NUM_MOTORS] = {0};
    float neutralAngles[NUM_MOTORS] = {0};
    bool zeroed = false;
    bool anglesInit = false;
    bool jumpGuardReset = false;

    void init() {
        computeNeutralAngles();
        zeroed = loadFromNVS();
    }

    void computeNeutralAngles() {
        Pose neutral = {0, 0, NEUTRAL_Z, 0, 0, 0};
        IKResult r = inverse_kinematics(neutral);
        for (int i = 0; i < NUM_MOTORS; i++)
            neutralAngles[i] = r.angles[i];
    }

    // ===== NVS 持久化 =====
    void saveToNVS() {
        Preferences prefs;
        prefs.begin("stewart", false);
        prefs.putBytes("zeroRaw64", zeroRaw, sizeof(zeroRaw));
        prefs.end();
    }

    bool loadFromNVS() {
        Preferences prefs;
        prefs.begin("stewart", true);
        size_t len = prefs.getBytes("zeroRaw64", zeroRaw, sizeof(zeroRaw));
        prefs.end();
        return len == sizeof(zeroRaw);
    }

    // ===== 唯一真值映射 =====
    // angle = MOTOR_SIGN × (raw - zeroRaw) × 360/16384 + 90
    float rawToAngle(int i, int64_t raw) const {
        int64_t d = raw - zeroRaw[i];
        return MOTOR_SIGN[i] * (float)d * 360.0f / 16384.0f + 90.0f;
    }

    // F5 座標轉換（相對於 enableAngle，與映射無關）
    int32_t angleToCoord(int i, float targetAngle, float enableAngle) const {
        float delta = targetAngle - enableAngle;
        return (int32_t)roundf(delta / (float)MOTOR_SIGN[i] * 16384.0f / 360.0f);
    }

    // ===== 每 cycle 更新角度（含跳變防護）=====
    void updateAngles(int64_t raw[NUM_MOTORS]) {
        if (jumpGuardReset) {
            anglesInit = false;
            jumpGuardReset = false;
        }
        for (int i = 0; i < NUM_MOTORS; i++) {
            if (raw[i] == INT64_MIN) continue;

            float newAngle = rawToAngle(i, raw[i]);

            if (anglesInit && fabsf(newAngle - angles[i]) > 15.0f) {
                Serial.printf("{\"warn\":\"M%d jump %.1f->%.1f, discarded\"}\n",
                              i + 1, angles[i], newAngle);
                continue;
            }
            angles[i] = newAngle;
        }
        anglesInit = true;
    }

    // ===== 校正操作 =====
    bool zeroAll(Servo42D& servos) {
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
        servos.flushReceiveBuffer();

        int64_t raw[NUM_MOTORS];
        int ok = servos.readAllRawEncoders(raw);
        for (int i = 0; i < NUM_MOTORS; i++) {
            if (raw[i] == INT64_MIN) return false;
        }

        for (int i = 0; i < NUM_MOTORS; i++) zeroRaw[i] = raw[i];
        zeroed = true;
        jumpGuardReset = true;
        saveToNVS();
        return true;
    }

    bool zeroMotor(int idx, Servo42D& servos) {
        int64_t val = servos.readRawEncoderValue(MOTOR_ADDR[idx]);
        if (val == INT64_MIN) return false;
        zeroRaw[idx] = val;
        jumpGuardReset = true;
        saveToNVS();
        return true;
    }
};
