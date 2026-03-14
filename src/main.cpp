#include <Arduino.h>
#include "servo42d.h"
#include "kinematics.h"

Servo42D servos;

int32_t zeroRaw[NUM_MOTORS] = {0};
float neutralAngles[NUM_MOTORS] = {0};
bool zeroed = false;

void computeNeutralAngles() {
    Pose neutral = {0, 0, NEUTRAL_Z, 0, 0, 0};
    IKResult r = inverse_kinematics(neutral);
    for (int i = 0; i < NUM_MOTORS; i++) {
        neutralAngles[i] = r.angles[i];
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    computeNeutralAngles();

    Serial.println("{\"status\":\"init\"}");

    if (!servos.begin()) {
        Serial.println("{\"error\":\"CAN init failed\"}");
        while (1) delay(1000);
    }

    // 確保馬達不出力
    servos.disableAll();

    Serial.println("{\"status\":\"ready\"}");
}

void loop() {
    // 檢查序列指令
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd == "Z") {
            // 全部歸零
            int32_t raw[NUM_MOTORS];
            servos.readAllEncoders(raw);
            for (int i = 0; i < NUM_MOTORS; i++) zeroRaw[i] = raw[i];
            zeroed = true;
            Serial.println("{\"status\":\"zeroed all\"}");
        } else if (cmd.startsWith("Z") && cmd.length() == 2) {
            // 單顆歸零: Z0~Z5
            int idx = cmd.charAt(1) - '0';
            if (idx >= 0 && idx < NUM_MOTORS) {
                int32_t val = servos.readEncoderRaw(MOTOR_ADDR[idx]);
                if (val >= 0) zeroRaw[idx] = val;
                Serial.printf("{\"status\":\"zeroed M%d\"}\n", idx + 1);
            }
        }
    }

    // 每 20ms 讀一次編碼器
    static uint32_t lastRead = 0;
    if (millis() - lastRead < 20) return;
    lastRead = millis();

    int32_t raw[NUM_MOTORS];
    int ok = servos.readAllEncoders(raw);

    // 計算馬達角度
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
        // 歸零時下腿朝上 = 90°，encoder delta 從那裡算起
        angles[i] = MOTOR_SIGN[i] * delta + 90.0f;
    }

    // 輸出 JSON
    Serial.printf("{\"a\":[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f],\"r\":[%ld,%ld,%ld,%ld,%ld,%ld],\"ok\":%d,\"z\":%d}\n",
        angles[0], angles[1], angles[2], angles[3], angles[4], angles[5],
        raw[0], raw[1], raw[2], raw[3], raw[4], raw[5],
        ok, zeroed ? 1 : 0);
}
