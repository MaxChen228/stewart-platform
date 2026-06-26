#pragma once
#include <Preferences.h>
#include "servo42d.h"
#include "kinematics.h"

// 編碼器映射策略：只用 MT6816 14-bit 單圈絕對位置。
// zeroRaw 存 NVS（int64 相容舊格式），運行時取低 14-bit 視為絕對。
// wrap 邊界置於 home 對面 ±180°，避開靜止雜訊。
// 限制：lower leg 工作範圍不可超過 home ±180°（實際 ±60° 內，遠夠用）。
struct EncoderState {
    int64_t zeroRaw[NUM_MOTORS] = {0};
    float angles[NUM_MOTORS] = {0};
    float neutralAngles[NUM_MOTORS] = {0};
    bool zeroed = false;

    void init() {
        computeNeutralAngles();
        // 預先把 angles 設成 home 假設值，避免某顆從未讀成功時顯示為 0°（leg 水平）
        for (int i = 0; i < NUM_MOTORS; i++) angles[i] = neutralAngles[i];
        zeroed = loadFromNVS();
    }

    void computeNeutralAngles() {
        Pose neutral = {0, 0, NEUTRAL_Z, 0, 0, 0};
        IKResult r = inverse_kinematics(neutral);
        // 中位姿態理論上必有解；若 IK invalid（幾何常數被改壞）退回 90° home 假設，
        // 不讓 r.angles 的未定義值（可能=0=leg 水平）污染 neutralAngles
        if (!r.valid) {
            for (int i = 0; i < NUM_MOTORS; i++) neutralAngles[i] = 90.0f;
            return;
        }
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

    // ===== 唯一真值映射（單圈絕對 + wrap）=====
    float rawToAngle(int i, int64_t raw) const {
        int32_t single = ((int32_t)(raw % 16384) + 16384) % 16384;
        int32_t zero   = ((int32_t)(zeroRaw[i] % 16384) + 16384) % 16384;
        int32_t delta  = single - zero;
        if (delta >=  8192) delta -= 16384;
        if (delta <  -8192) delta += 16384;
        return MOTOR_SIGN[i] * (float)delta * 360.0f / 16384.0f + 90.0f;
    }

    // F5 座標轉換（相對於 enableAngle，與 zeroRaw 無關）
    int32_t angleToCoord(int i, float targetAngle, float enableAngle) const {
        float delta = targetAngle - enableAngle;
        return (int32_t)roundf(delta / (float)MOTOR_SIGN[i] * 16384.0f / 360.0f);
    }

    void updateAngles(int64_t raw[NUM_MOTORS]) {
        for (int i = 0; i < NUM_MOTORS; i++) {
            if (raw[i] == INT64_MIN) continue;
            angles[i] = rawToAngle(i, raw[i]);
        }
    }

    // ===== 校正操作 =====
    // 必須在實體 home 姿態（所有下腿朝上 = 90°）執行一次，永久有效。
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
        servos.readAllRawEncoders(raw);
        for (int i = 0; i < NUM_MOTORS; i++) {
            if (raw[i] == INT64_MIN) return false;
        }

        for (int i = 0; i < NUM_MOTORS; i++) zeroRaw[i] = raw[i];
        zeroed = true;
        saveToNVS();
        return true;
    }

    bool zeroMotor(int idx, Servo42D& servos) {
        int64_t val = servos.readRawEncoderValue(MOTOR_ADDR[idx]);
        if (val == INT64_MIN) return false;
        zeroRaw[idx] = val;
        saveToNVS();
        return true;
    }
};
