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
int32_t sessionZeroRaw[NUM_MOTORS] = {0}; // 0x92 後調整的 zeroRaw（不存 NVS）

uint16_t posSpeed = 30;   // 最大移動速度 (RPM)
uint8_t  posAcc   = 5;    // 加減速參數

constexpr float SMOOTH_TIME = 0.5f; // setpoint 平滑時間（秒）

// ===== 自適應追蹤 =====
float prevAngles[NUM_MOTORS] = {0};  // 上一 cycle 的角度（算速度用）
float trackingMu = 0.5f;            // 震動懲罰係數（越大越優先穩定）
float trackingKd = 3.0f;            // 速度阻尼係數（僅在過衝時作用）
float smoothKinetic = 0;            // 低通濾波後的動能（偵測持續震盪）
bool adaptiveFirstCycle = true;

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
// Enable 時 0x92 設零 → coordBase=0 → coord 直接是相對 enable 位置的偏移
int32_t angleToCoord(int motorIdx, float targetAngle) {
    float deltaAngle = targetAngle - motorZeroAngle[motorIdx];
    return (int32_t)roundf(deltaAngle / (float)MOTOR_SIGN[motorIdx] * 16384.0f / 360.0f);
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    computeNeutralAngles();
    zeroed = loadZeroFromNVS();
    // sessionZeroRaw 初始化為 NVS 的 zeroRaw（0x92 呼叫前兩者相同）
    for (int i = 0; i < NUM_MOTORS; i++) sessionZeroRaw[i] = zeroRaw[i];

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
        for (int i = 0; i < NUM_MOTORS; i++) {
            zeroRaw[i] = raw[i];
            sessionZeroRaw[i] = raw[i]; // 同步，此刻兩者一致
        }
        zeroed = true;
        saveZeroToNVS();
        Serial.println("{\"status\":\"zeroed all\",\"saved\":true}");
    } else if (cmd.startsWith("Z") && cmd.length() == 2) {
        int idx = cmd.charAt(1) - '0';
        if (idx >= 0 && idx < NUM_MOTORS) {
            int32_t val = servos.readEncoderRaw(MOTOR_ADDR[idx]);
            if (val >= 0) {
                zeroRaw[idx] = val;
                sessionZeroRaw[idx] = val;
            }
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

        // 1. 讀取 0x92 前的 encoder
        int32_t rawBefore[NUM_MOTORS];
        servos.readAllEncoders(rawBefore);

        // 2. 啟用馬達 + 設定零點
        for (int i = 0; i < NUM_MOTORS; i++)
            servos.setEnable(MOTOR_ADDR[i], true);
        delay(20);
        for (int i = 0; i < NUM_MOTORS; i++) {
            servos.setZeroPoint(MOTOR_ADDR[i]);
            delay(10);
        }

        // 3. 讀取 0x92 後的 encoder，計算偏移量
        int32_t rawAfter[NUM_MOTORS];
        servos.readAllEncoders(rawAfter);
        for (int i = 0; i < NUM_MOTORS; i++) {
            int32_t shift = rawAfter[i] - rawBefore[i];
            if (shift > 8192) shift -= 16384;
            if (shift < -8192) shift += 16384;
            // 調整 session zeroRaw，使角度計算結果不變
            sessionZeroRaw[i] = ((int32_t)zeroRaw[i] + shift + 16384) % 16384;
        }

        // 4. 用調整後的 zeroRaw 算當前角度
        for (int i = 0; i < NUM_MOTORS; i++) {
            int32_t d = rawAfter[i] - sessionZeroRaw[i];
            if (d > 8192) d -= 16384;
            if (d < -8192) d += 16384;
            motorZeroAngle[i] = MOTOR_SIGN[i] * (float)d * 360.0f / 16384.0f + 90.0f;
        }

        // 5. 初始化自適應追蹤
        smoothedTarget = targetPose;
        for (int i = 0; i < NUM_MOTORS; i++)
            prevAngles[i] = motorZeroAngle[i];
        adaptiveFirstCycle = true;
        posEnabled = true;

        Serial.printf("{\"status\":\"pos enabled\",\"speed\":%d,\"acc\":%d,\"angles\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]}\n",
            posSpeed, posAcc,
            motorZeroAngle[0],motorZeroAngle[1],motorZeroAngle[2],
            motorZeroAngle[3],motorZeroAngle[4],motorZeroAngle[5]);
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
    } else if (cmd.startsWith("M ")) {
        // 設定自適應追蹤參數: M mu [kd]
        float mu, kd = -1;
        int n = sscanf(cmd.c_str(), "M %f %f", &mu, &kd);
        if (n >= 1) {
            trackingMu = fmaxf(0.0f, mu);
            if (n >= 2) trackingKd = fmaxf(0.0f, kd);
            Serial.printf("{\"status\":\"tracking params\",\"mu\":%.2f,\"kd\":%.2f}\n", trackingMu, trackingKd);
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

    // 清掉上一 cycle 的 F5 回覆，避免塞爆 MCP2515 buffer
    servos.flushReceiveBuffer();

    int32_t raw[NUM_MOTORS];
    int ok = servos.readAllEncoders(raw);

    // 計算當前馬達角度（posEnabled 時用 sessionZeroRaw，否則用 zeroRaw）
    static float angles[NUM_MOTORS] = {0};  // static: 讀取失敗時保持上一次值
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (raw[i] < 0) {
            // 保持上一次的 angles[i]，不覆蓋為 neutralAngle
            continue;
        }
        int32_t ref = sessionZeroRaw[i];
        int32_t d = raw[i] - ref;
        if (d > 8192) d -= 16384;
        if (d < -8192) d += 16384;
        float delta = (float)d * 360.0f / 16384.0f;
        angles[i] = MOTOR_SIGN[i] * delta + 90.0f;
    }

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
            // 初始化 prevAngles（第一個 cycle）
            if (adaptiveFirstCycle) {
                for (int i = 0; i < NUM_MOTORS; i++)
                    prevAngles[i] = angles[i];
                smoothKinetic = 0;
                adaptiveFirstCycle = false;
            }

            // 計算逐馬達速度 + 整體震動指標
            float vel[NUM_MOTORS];
            float kinetic = 0;
            for (int i = 0; i < NUM_MOTORS; i++) {
                vel[i] = angles[i] - prevAngles[i]; // 度/cycle
                kinetic += vel[i] * vel[i];
            }

            // 低通濾波動能：偵測持續震盪，忽略單次大移動
            // α=0.1 → 時間常數 200ms，持續震盪 >300ms 才大幅壓 gain
            smoothKinetic = 0.9f * smoothKinetic + 0.1f * kinetic;
            gain = 1.0f / (1.0f + trackingMu * smoothKinetic);

            for (int i = 0; i < NUM_MOTORS; i++) {
                targetAngles[i] = target.angles[i];

                float error = target.angles[i] - angles[i];
                // 方向性阻尼：僅在過衝時煞車（vel 方向與 error 相反）
                // 朝目標移動時不阻尼 → 大幅移動不再 chatter
                float damp = (error * vel[i] < 0) ? trackingKd * vel[i] : 0.0f;
                adjustedAngles[i] = angles[i] + gain * error - damp;

                coords[i] = angleToCoord(i, adjustedAngles[i]);

                // 安全限制
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
                  "\"r\":[%ld,%ld,%ld,%ld,%ld,%ld],"
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
