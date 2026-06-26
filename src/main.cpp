#include <Arduino.h>
#include <SPI.h>
#include "servo42d.h"
#include "kinematics.h"
#include "encoder.h"
#include "forward_kinematics.h"
#include "control.h"

Servo42D servos;
EncoderState enc;

// ===== 位置模式控制 =====
Pose targetPose = {0, 0, NEUTRAL_Z, 0, 0, 0};

// ===== Auto-return（item 1）：馬達定時主動上報，ESP32 連續排空、控制環只取快照 =====
// MCP2515 只有 2 RX buffer，6 顆 streaming 必須「高頻連續排空」才不溢位——靠 loop()
// 本身快速空轉（20ms gate 只節流控制，不節流 loop 迭代）達成，無需 ISR/task。
// 預設關閉：行為與輪詢版完全相同，燒錄安全。需馬達上電後 A1 啟用才生效。
int64_t latestRaw[NUM_MOTORS];
bool autoReturnMode = false;
uint16_t arPeriodMs = 3;              // 馬達主動上報週期，可調（AR 指令）

// ===== 可調速率（item 2 of 本輪）=====
uint32_t loopPeriodUs = 20000;       // 控制迴圈週期，可調（L 指令，單位 ms→µs）
const uint32_t TELE_PERIOD_MS = 30;  // 遙測輸出週期（與控制解耦，固定 ~33Hz，免塞爆 115200 序列埠）
bool posEnabled = false;
float enableAngle[NUM_MOTORS] = {0};

uint16_t posSpeed = 120;
uint8_t  posAcc   = 100;

// ===== HOLD 模式 =====
// 純 passthrough：snapshot 當前 enc.angles，每 cycle 直送 F5，跳過 PD/IK/平滑
// 用來測馬達內環硬度
bool holdMode = false;
float holdAngles[NUM_MOTORS] = {0};
float maxHoldErr = 0;

// 擾動注入器（U 指令）：HOLD 模式下對單顆馬達的 F5 目標短暫加偏移 → 馬達主動衝出再彈回 = 脈衝
float    bumpDeg[NUM_MOTORS] = {0};   // 六軸脈衝偏移（enc 慣例度）；U=單軸填一格、W=六軸全填
uint32_t bumpUntilMs = 0;             // 脈衝結束時刻（millis）；過期由 hold 迴圈歸零

// 姿態調控（G 指令）：HOLD 下把 holdAngles 從 holdBase 平滑移到 holdBase+Δ（升降/旋轉/平移）。
// 保留 HOLD 直送機制 → 馬達純 P 死咬移動中的目標 = 穩定性不變，只是目標會動。
// host 用 kin.js IK 算 Δenc（同 W 的 deltaAngle×MOTOR_SIGN）；Δ 相對 H 基準、不累積。
float    holdBase[NUM_MOTORS]   = {0};   // H 鎖位時的基準姿態（不變，G 的 Δ 相對它）
float    holdTarget[NUM_MOTORS] = {0};   // 移動目標 = holdBase + Δ
float    holdStart[NUM_MOTORS]  = {0};   // 本次移動起點（發 G 當下的 holdAngles）
uint32_t holdMoveStart = 0, holdMoveMs = 0;   // 移動起始時刻、總時長（0=不在移動）

// 開機預設 vFOC PID：純P工作點 [1024,0,0,0]（2026-06-26 定案）。
// Ki=0 滅自持極限環根因（積分撞死區）；maxKp 補剛性；F5 posSpeed 提供阻尼；
// Kd=0 因離散微分=高頻放大器→嘯叫，Kv=0 不需。runtime K 指令仍可覆蓋微調。
// 不靠馬達 EEPROM，版本控制在韌體。詳見記憶 project_pid_working_point。
constexpr uint16_t BOOT_KP = 1024, BOOT_KI = 0, BOOT_KD = 0, BOOT_KV = 0;

// 當前 vFOC PID（單一真相源）：開機=BOOT 值，收 K 指令更新，telemetry 上報 → 前端滑桿反映真值不寫死
uint16_t curKp = BOOT_KP, curKi = BOOT_KI, curKd = BOOT_KD, curKv = BOOT_KV;

// ===== 控制模式 =====
// 0 = joint-space（舊版自適應追蹤）
// 1 = task-space PD（FK + IK）
int controlMode = 0;
int silentBootTotal = 0;
int silentBootPerId[7] = {0};

// Task-space controller
TaskSpacePD tsController;

// Joint-space 備用參數
Pose smoothedTarget = {0, 0, NEUTRAL_Z, 0, 0, 0};
constexpr float SMOOTH_TIME = 0.5f;
float prevAngles[NUM_MOTORS] = {0};
float trackingMu = 0.5f;
float trackingKd = 3.0f;
float maxGain = 0.3f;
float smoothKinetic = 0;
bool adaptiveFirstCycle = true;

void posStop() {
    posEnabled = false;
    holdMode = false;
    servos.stopAllPosition(5);
}

void posDisable() {
    posEnabled = false;
    holdMode = false;
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

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    delay(200); // 給 FreeRTOS / SPI mutex 完全初始化的時間，避免 paramLock NULL

    // 明確 init SPI bus（VSPI 預設腳位 SCK=18 MISO=19 MOSI=23 SS=5）
    // 必須在任何 MCP_CAN SPI 操作前呼叫，確保 SPI 的 paramLock mutex 已建立
    SPI.begin(18, 19, 23, 5);
    SPI.setFrequency(10000000);
    delay(50);

    enc.init();

    Serial.printf("{\"status\":\"init\",\"calibrated\":%s}\n",
                  enc.zeroed ? "true" : "false");

    while (!servos.begin()) {
        Serial.println("{\"error\":\"CAN init failed, retrying...\"}");
        delay(2000);
    }

    // DIAGNOSTIC: 不送任何 cmd，只 begin CAN + listen 5s，看 motor 是否從通電就自發 spam
    {
        uint32_t st = millis();
        while (millis() - st < 5000) {
            if (servos.can.checkReceive() == CAN_MSGAVAIL) {
                unsigned long id; uint8_t len; uint8_t buf[8];
                servos.can.readMsgBuf(&id, &len, buf);
                silentBootTotal++;
                if (id >= 1 && id <= 6) silentBootPerId[id]++;
            }
        }
    }
    // 之後恢復一般流程
    servos.disableAll();
    for (int i = 0; i < NUM_MOTORS; i++) {
        servos.setResponseMode(MOTOR_ADDR[i], 1, 0);
        delay(10);
    }

    // 開機持久化 vFOC PID：純P 工作點 [1024,0,0,0]（見上 BOOT_* 定義）。
    // 重開機馬達回出廠值 → 每次開機重設成韌體版控的值，不靠馬達 EEPROM。
    for (int i = 0; i < NUM_MOTORS; i++) {
        servos.setVFOC_KpKi(MOTOR_ADDR[i], BOOT_KP, BOOT_KI);
        delay(10);
        servos.setVFOC_KdKv(MOTOR_ADDR[i], BOOT_KD, BOOT_KV);
        delay(10);
    }

    delay(200);
    servos.flushReceiveBuffer();

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
    } else if (cmd == "A1") {
        // 啟用 auto-return：6 顆每 arPeriodMs 主動上報 0x35 位置
        for (int i = 0; i < NUM_MOTORS; i++) latestRaw[i] = INT64_MIN;
        int okc = 0;
        for (int i = 0; i < NUM_MOTORS; i++) {
            if (servos.setAutoReturn(MOTOR_ADDR[i], 0x35, arPeriodMs)) okc++;
            delay(5);
        }
        servos.flushReceiveBuffer();
        autoReturnMode = true;
        Serial.printf("{\"status\":\"auto-return ON\",\"period_ms\":%d,\"cfg_ok\":%d}\n",
                      arPeriodMs, okc);
    } else if (cmd.startsWith("AR ")) {
        // 調整上報週期（ms）；若已啟用則即時重設
        arPeriodMs = constrain(cmd.substring(3).toInt(), 1, 100);
        if (autoReturnMode)
            for (int i = 0; i < NUM_MOTORS; i++) { servos.setAutoReturn(MOTOR_ADDR[i], 0x35, arPeriodMs); delay(5); }
        Serial.printf("{\"status\":\"ar period\",\"period_ms\":%d}\n", arPeriodMs);
    } else if (cmd.startsWith("C ")) {
        // 設回覆模式：C 0 0=不回覆 / C 1 0=只即時 / C 1 1=預設全回
        int sp = cmd.indexOf(' ', 2);
        int xx = cmd.substring(2, sp < 0 ? cmd.length() : sp).toInt();
        int yy = sp < 0 ? 0 : cmd.substring(sp + 1).toInt();
        for (int i = 0; i < NUM_MOTORS; i++) { servos.setResponseMode(MOTOR_ADDR[i], xx, yy); delay(5); }
        servos.flushReceiveBuffer();
        Serial.printf("{\"status\":\"resp mode\",\"xx\":%d,\"yy\":%d}\n", xx, yy);
    } else if (cmd.startsWith("L ")) {
        // 調整控制迴圈週期（ms）
        uint32_t ms = constrain(cmd.substring(2).toInt(), 1, 100);
        loopPeriodUs = ms * 1000;
        Serial.printf("{\"status\":\"loop period\",\"period_ms\":%u,\"hz\":%.0f}\n", ms, 1000.0f / ms);
    } else if (cmd == "A0") {
        for (int i = 0; i < NUM_MOTORS; i++) {
            servos.setAutoReturn(MOTOR_ADDR[i], 0x35, 0);  // 0 = 停用上報
            delay(5);
        }
        autoReturnMode = false;
        Serial.println("{\"status\":\"auto-return OFF (回輪詢)\"}");
    } else if (cmd == "D") {
        posDisable();
        Serial.println("{\"status\":\"disabled all\"}");
    } else if (cmd == "E") {
        if (!enc.zeroed) {
            Serial.println("{\"error\":\"not calibrated, run Z first\"}");
            return;
        }

        uint8_t enMask = servos.enableAll();
        int enCnt = __builtin_popcount(enMask);
        delay(20);
        for (int i = 0; i < NUM_MOTORS; i++) {
            servos.setZeroPoint(MOTOR_ADDR[i]);
            delay(10);
            servos.flushReceiveBuffer();
        }

        int64_t raw[NUM_MOTORS];
        servos.readAllRawEncoders(raw);
        enc.updateAngles(raw);   // 0x35 不受 0x92 影響，先刷新 enc.angles 當 fallback 基準
        for (int i = 0; i < NUM_MOTORS; i++) {
            // 讀失敗 fallback 用新鮮 enc.angles（真實當前角），不用寫死 90°，
            // 否則 0x92 已把零點設在真實位置 → 首個 F5 coord 算錯 → 該軸跳動（與 H handler 對齊）
            enableAngle[i] = (raw[i] != INT64_MIN)
                ? enc.rawToAngle(i, raw[i]) : enc.angles[i];
        }

        // 初始化控制器：target 自動對齊當前 FK 姿態（避免 Enable 瞬間暴衝）
        if (controlMode == 1) {
            tsController.reset(enc.angles);
            targetPose = tsController.currentPose; // Enable 時 target = 當前位置
        } else {
            smoothedTarget = targetPose;
            for (int i = 0; i < NUM_MOTORS; i++)
                prevAngles[i] = enableAngle[i];
            adaptiveFirstCycle = true;
        }

        posEnabled = true;
        Serial.printf("{\"status\":\"pos enabled\",\"mode\":%d,\"speed\":%d,\"acc\":%d,"
            "\"enOk\":[%d,%d,%d,%d,%d,%d],\"enCnt\":%d}\n",
            controlMode, posSpeed, posAcc,
            (enMask>>0)&1,(enMask>>1)&1,(enMask>>2)&1,(enMask>>3)&1,(enMask>>4)&1,(enMask>>5)&1,
            enCnt);
    } else if (cmd == "S") {
        posStop();
        Serial.println("{\"status\":\"pos stopped\"}");
    } else if (cmd == "H") {
        // HOLD: snapshot 當前角度，enable + 0x92 + 鎖位
        if (!enc.zeroed) {
            Serial.println("{\"error\":\"not calibrated, run Z first\"}");
            return;
        }
        // 先確保 enc.angles 是新鮮的
        servos.flushReceiveBuffer();
        int64_t raw0[NUM_MOTORS];
        servos.readAllRawEncoders(raw0);
        enc.updateAngles(raw0);
        for (int i = 0; i < NUM_MOTORS; i++) holdAngles[i] = enc.angles[i];
        for (int i = 0; i < NUM_MOTORS; i++) holdBase[i] = holdAngles[i];   // G 的 Δ 基準
        holdMoveMs = 0;                                                     // 取消任何進行中的姿態移動

        // Enable 馬達 + 設零點
        uint8_t enMask = servos.enableAll();
        int enCnt = __builtin_popcount(enMask);
        delay(20);
        for (int i = 0; i < NUM_MOTORS; i++) {
            servos.setZeroPoint(MOTOR_ADDR[i]);
            delay(10);
            servos.flushReceiveBuffer();
        }

        // 重抓 enableAngle（0x92 之後的當前角度）
        int64_t raw[NUM_MOTORS];
        servos.readAllRawEncoders(raw);
        for (int i = 0; i < NUM_MOTORS; i++) {
            enableAngle[i] = (raw[i] != INT64_MIN)
                ? enc.rawToAngle(i, raw[i]) : holdAngles[i];
        }

        holdMode = true;
        posEnabled = true;
        maxHoldErr = 0;
        Serial.printf("{\"status\":\"hold\",\"hold\":[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f],"
            "\"enOk\":[%d,%d,%d,%d,%d,%d],\"enCnt\":%d}\n",
            holdAngles[0],holdAngles[1],holdAngles[2],
            holdAngles[3],holdAngles[4],holdAngles[5],
            (enMask>>0)&1,(enMask>>1)&1,(enMask>>2)&1,(enMask>>3)&1,(enMask>>4)&1,(enMask>>5)&1,
            enCnt);
    } else if (cmd.startsWith("J ")) {
        int mA;
        if (sscanf(cmd.c_str(), "J %d", &mA) == 1) {
            mA = constrain(mA, 100, 3000);
            int okMask[NUM_MOTORS] = {0};
            int okCnt = 0;
            for (int i = 0; i < NUM_MOTORS; i++) {
                bool ok = servos.setWorkingCurrent(MOTOR_ADDR[i], (uint16_t)mA);
                okMask[i] = ok ? 1 : 0;
                if (ok) okCnt++;
                delay(8);
                servos.flushReceiveBuffer();
            }
            Serial.printf("{\"tune\":\"current\",\"mA\":%d,\"ok\":[%d,%d,%d,%d,%d,%d],\"okCnt\":%d}\n",
                mA, okMask[0],okMask[1],okMask[2],okMask[3],okMask[4],okMask[5], okCnt);
        }
    } else if (cmd.startsWith("P ")) {
        float v[6];
        if (sscanf(cmd.c_str(), "P %f %f %f %f %f %f", &v[0],&v[1],&v[2],&v[3],&v[4],&v[5]) == 6) {
            targetPose = {v[0], v[1], v[2], v[3], v[4], v[5]};
            Serial.printf("{\"status\":\"target set\",\"t\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]}\n",
                v[0],v[1],v[2],v[3],v[4],v[5]);
        }
    } else if (cmd.startsWith("CM ")) {
        // 控制模式切換：CM 0 = joint-space, CM 1 [kp kd] = task-space PD
        // （前綴從 "C " 改為 "CM "：避開上方 0x8C 回覆模式 "C " 的撞名遮蔽）
        float mode_f, kp = -1, kd = -1;
        int n = sscanf(cmd.c_str(), "CM %f %f %f", &mode_f, &kp, &kd);
        if (n >= 1) {
            controlMode = (int)mode_f;
            if (controlMode == 1 && n >= 2) {
                for (int i = 0; i < 6; i++) tsController.Kp[i] = fmaxf(0.01f, fminf(1.0f, kp));
                if (n >= 3) for (int i = 0; i < 6; i++) tsController.Kd[i] = fmaxf(0.0f, kd);
            }
            Serial.printf("{\"status\":\"control mode\",\"mode\":%d,\"kp\":%.3f,\"kd\":%.3f}\n",
                controlMode, tsController.Kp[0], tsController.Kd[0]);
        }
    } else if (cmd.startsWith("K ")) {
        int kp, ki, kd, kv;
        if (sscanf(cmd.c_str(), "K %d %d %d %d", &kp, &ki, &kd, &kv) == 4) {
            int okMask[NUM_MOTORS] = {0};
            int okCnt = 0;
            for (int i = 0; i < NUM_MOTORS; i++) {
                bool a = servos.setVFOC_KpKi(MOTOR_ADDR[i], kp, ki);
                delay(8);
                servos.flushReceiveBuffer();
                bool b = servos.setVFOC_KdKv(MOTOR_ADDR[i], kd, kv);
                delay(8);
                servos.flushReceiveBuffer();
                okMask[i] = (a && b) ? 1 : 0;
                if (okMask[i]) okCnt++;
            }
            curKp = kp; curKi = ki; curKd = kd; curKv = kv;   // 更新 SoT 當前 PID
            Serial.printf("{\"tune\":\"pid\",\"kp\":%d,\"ki\":%d,\"kd\":%d,\"kv\":%d,"
                "\"ok\":[%d,%d,%d,%d,%d,%d],\"okCnt\":%d}\n",
                kp, ki, kd, kv,
                okMask[0],okMask[1],okMask[2],okMask[3],okMask[4],okMask[5], okCnt);
        }
    } else if (cmd.startsWith("V ")) {
        int spd, ac;
        if (sscanf(cmd.c_str(), "V %d %d", &spd, &ac) == 2) {
            posSpeed = constrain(spd, 1, 200);
            posAcc = constrain(ac, 1, 255);
            // V 是 ESP32 本地變數，下個 cycle 才會用到 → 永遠 ok（無 CAN 動作）
            Serial.printf("{\"tune\":\"motion\",\"speed\":%d,\"acc\":%d,"
                "\"ok\":[1,1,1,1,1,1],\"okCnt\":6,\"local\":1}\n", posSpeed, posAcc);
        }
    } else if (cmd.startsWith("M ")) {
        // Joint-space 參數（C 0 模式用）
        float mu, kd = -1, mg = -1;
        int n = sscanf(cmd.c_str(), "M %f %f %f", &mu, &kd, &mg);
        if (n >= 1) {
            trackingMu = fmaxf(0.0f, mu);
            if (n >= 2) trackingKd = fmaxf(0.0f, kd);
            if (n >= 3) maxGain = fmaxf(0.05f, fminf(1.0f, mg));
            Serial.printf("{\"status\":\"tracking params\",\"mu\":%.2f,\"kd\":%.2f,\"maxGain\":%.2f}\n",
                trackingMu, trackingKd, maxGain);
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
    } else if (cmd.startsWith("U ")) {
        // 擾動脈衝（單軸）：U <motor 0-5> <deg> <ms> — HOLD 模式下對該馬達 F5 目標加 deg 偏移持續 ms，再自動歸零
        int idx, ms; float deg;
        if (sscanf(cmd.c_str(), "U %d %f %d", &idx, &deg, &ms) == 3 && idx >= 0 && idx < NUM_MOTORS) {
            for (int i = 0; i < NUM_MOTORS; i++) bumpDeg[i] = 0;
            bumpDeg[idx] = deg;
            bumpUntilMs = millis() + constrain(ms, 1, 5000);
            Serial.printf("{\"status\":\"bump\",\"motor\":%d,\"deg\":%.2f,\"ms\":%d,\"hold\":%d}\n",
                idx + 1, deg, ms, holdMode ? 1 : 0);
        }
    } else if (cmd.startsWith("W ")) {
        // 擾動脈衝（六軸協同）：W <d0> <d1> <d2> <d3> <d4> <d5> <ms> — HOLD 模式下六軸同時加各自偏移持續 ms
        // host 端 disturb.js 用 task-space IK 算各軸 deg（已乘 MOTOR_SIGN 轉 enc 慣例），這裡只照單全收
        float d[NUM_MOTORS]; int ms;
        if (sscanf(cmd.c_str(), "W %f %f %f %f %f %f %d",
                   &d[0], &d[1], &d[2], &d[3], &d[4], &d[5], &ms) == 7) {
            for (int i = 0; i < NUM_MOTORS; i++) bumpDeg[i] = constrain(d[i], -30.0f, 30.0f);
            bumpUntilMs = millis() + constrain(ms, 1, 5000);
            Serial.printf("{\"status\":\"bumpW\",\"deg\":[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f],\"ms\":%d,\"hold\":%d}\n",
                bumpDeg[0], bumpDeg[1], bumpDeg[2], bumpDeg[3], bumpDeg[4], bumpDeg[5], ms, holdMode ? 1 : 0);
        }
    } else if (cmd.startsWith("G ")) {
        // 姿態調控（六軸協同持久移動）：G <d0..d5> <ms> — HOLD 下把 holdAngles 從 holdBase 平滑移到 holdBase+Δ
        // host 用 kin.js IK 算 Δenc（同 W 的 deltaAngle×MOTOR_SIGN）；Δ 相對 H 基準、不累積。ms 內 smoothstep 加減速。
        float d[NUM_MOTORS]; int ms;
        if (sscanf(cmd.c_str(), "G %f %f %f %f %f %f %d",
                   &d[0], &d[1], &d[2], &d[3], &d[4], &d[5], &ms) == 7) {
            for (int i = 0; i < NUM_MOTORS; i++) {
                holdStart[i]  = holdAngles[i];                       // 從當下位置起步（多段 G 可接續）
                holdTarget[i] = holdBase[i] + constrain(d[i], -40.0f, 40.0f);
            }
            holdMoveStart = millis();
            holdMoveMs = constrain(ms, 1, 10000);
            Serial.printf("{\"status\":\"goto\",\"d\":[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f],\"ms\":%d,\"hold\":%d}\n",
                d[0], d[1], d[2], d[3], d[4], d[5], ms, holdMode ? 1 : 0);
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
        Serial.printf("],\"mode\":%d,\"posEnabled\":%d}}\n", controlMode, posEnabled ? 1 : 0);
    } else if (cmd == "B") {
        Serial.printf("{\"silent_boot\":{\"total\":%d,\"per_id\":[%d,%d,%d,%d,%d,%d]}}\n",
                      silentBootTotal,
                      silentBootPerId[1],silentBootPerId[2],silentBootPerId[3],
                      silentBootPerId[4],silentBootPerId[5],silentBootPerId[6]);
    } else if (cmd == "X") {
        // 深度診斷已完成（根因 = 某顆 motor firmware auto-broadcast bug，~78% reply rate）
        // 保留輕量化版供日後快速健診：只跑 listen + 6 顆 baseline
        bool savedPos = posEnabled, savedHold = holdMode;
        posEnabled = false; holdMode = false;
        delay(100);
        servos.flushReceiveBuffer();
        Serial.println("{\"xdiag\":\"start\"}");

        // 三段對比：normal/active(ACK) vs listen-only(no-ACK) vs filter to id=99
        auto countListen = [&](uint32_t ms) -> int {
            uint32_t st = millis();
            int n = 0;
            while (millis() - st < ms) {
                if (servos.can.checkReceive() == CAN_MSGAVAIL) {
                    unsigned long id; uint8_t len; uint8_t buf[8];
                    servos.can.readMsgBuf(&id, &len, buf);
                    n++;
                }
            }
            return n;
        };

        // (a) normal mode 基準
        int normalRx = countListen(1500);

        // (b) listen-only mode — 我們不 ACK 任何東西，看 spam 是否消失
        bool lo = servos.can.setListenOnly();
        int listenOnlyRx = countListen(1500);

        // SANITY 3: 5 次 write-read 看是否穩定 + mode change 行為
        servos.can.setNormal();
        servos.can.rawSetMode(0x80);
        for (int t = 0; t < 5; t++) {
            uint8_t pat = 0x55 << (t & 1);  // 0x55 / 0xAA alternating
            servos.can.rawWriteReg(0x20, pat);
            uint8_t r1 = servos.can.rawReadReg(0x20);
            uint8_t r2 = servos.can.rawReadReg(0x20);  // 再讀一次
            Serial.printf("{\"san3_inconfig\":{\"t\":%d,\"wrote\":\"0x%02X\",\"r1\":\"0x%02X\",\"r2\":\"0x%02X\"}}\n",
                          t, pat, r1, r2);
        }
        // 寫 0xAA，然後切 normal，再讀
        servos.can.rawWriteReg(0x20, 0xAA);
        uint8_t conf_r = servos.can.rawReadReg(0x20);
        servos.can.rawSetMode(0x00);
        uint8_t norm_r1 = servos.can.rawReadReg(0x20);
        uint8_t norm_r2 = servos.can.rawReadReg(0x20);
        Serial.printf("{\"san3_modechange\":{\"conf\":\"0x%02X\",\"norm1\":\"0x%02X\",\"norm2\":\"0x%02X\"}}\n",
                      conf_r, norm_r1, norm_r2);

        // TEST 1: dynamic filter to id=3, listen 1.5s, check id distribution
        servos.can.rawSetHardwareFilter(3);
        uint32_t lstart = millis();
        int idCnt[8] = {0};
        int filterRx = 0;
        while (millis() - lstart < 1500) {
            if (servos.can.checkReceive() == CAN_MSGAVAIL) {
                unsigned long id; uint8_t len; uint8_t buf[8];
                servos.can.readMsgBuf(&id, &len, buf);
                filterRx++;
                if (id < 8) idCnt[id]++;
            }
        }
        Serial.printf("{\"dyn_filter_to_3\":{\"total\":%d,\"per_id\":[%d,%d,%d,%d,%d,%d,%d,%d]}}\n",
                      filterRx, idCnt[0],idCnt[1],idCnt[2],idCnt[3],idCnt[4],idCnt[5],idCnt[6],idCnt[7]);

        // TEST 2: per-motor 100 query with per-query dynamic filter
        // (filter changes before each query to target motor)
        Serial.println("{\"xdiag\":\"per_motor_with_dyn_filter\"}");
        for (int i = 0; i < NUM_MOTORS; i++) {
            servos.can.rawSetHardwareFilter(MOTOR_ADDR[i]);
            servos.flushReceiveBuffer();
            int ok=0;
            for (int k = 0; k < 100; k++) {
                if (servos.readRawEncoderValue(MOTOR_ADDR[i], 20) != INT64_MIN) ok++;
            }
            Serial.printf("{\"xdiag\":\"dyn_m%d\",\"ok\":%d}\n", i+1, ok);
        }
        servos.can.rawSetFilterAcceptAll();

        Serial.printf("{\"xdiag\":\"bus_test\",\"normal_rx\":%d,\"listen_only_rx\":%d,\"filter99_rx\":%d,"
                      "\"lo_ok\":%d,\"f_ok\":%d}\n",
                      normalRx, listenOnlyRx, filterRx, lo?1:0, 1);
        Serial.printf("{\"xdiag\":\"spam\",\"baseline\":%d}\n", normalRx);

        // 失敗分類：nf=完全沒收到 frame, wm=收到別顆 frame, wc=對的 motor 錯 cmd
        servos.flushReceiveBuffer();
        for (int i = 0; i < NUM_MOTORS; i++) {
            int ok=0, nf=0, wm=0, wc=0, cf=0;
            for (int k = 0; k < 100; k++) {
                uint8_t txCmd[2] = {0x35, (uint8_t)((MOTOR_ADDR[i]+0x35) & 0xFF)};
                if (servos.can.sendMsgBuf(MOTOR_ADDR[i], 0, 2, txCmd) != CAN_OK) { nf++; continue; }
                uint32_t t0 = micros();
                bool got=false, sawAny=false;
                int lwm=0, lwc=0, lcf=0;
                while ((micros()-t0) < 30000) {
                    if (servos.can.checkReceive() == CAN_MSGAVAIL) {
                        sawAny = true;
                        unsigned long rxId; uint8_t rxLen; uint8_t rxBuf[8];
                        servos.can.readMsgBuf(&rxId, &rxLen, rxBuf);
                        if ((rxId & 0x7FF) != MOTOR_ADDR[i]) { lwm++; continue; }
                        if (rxLen < 1 || rxBuf[0] != 0x35) { lwc++; continue; }
                        if (rxLen < 8) { lcf++; continue; }
                        uint8_t crc = MOTOR_ADDR[i];
                        for (int j = 0; j < 7; j++) crc += rxBuf[j];
                        if ((crc & 0xFF) != rxBuf[7]) { lcf++; continue; }
                        ok++; got=true; break;
                    }
                }
                if (!got) {
                    if (!sawAny) nf++;
                    else if (lwm) wm++;
                    else if (lwc) wc++;
                    else cf++;
                }
            }
            Serial.printf("{\"xdiag\":\"m%d\",\"ok\":%d,\"nf\":%d,\"wm\":%d,\"wc\":%d,\"cf\":%d}\n",
                          i+1, ok, nf, wm, wc, cf);
        }

        // 再 listen 1s 看 query 過程是否誘發 spam
        servos.flushReceiveBuffer();
        uint32_t lstart2 = millis();
        int idCnt2[7] = {0};
        while (millis() - lstart2 < 1000) {
            if (servos.can.checkReceive() == CAN_MSGAVAIL) {
                unsigned long id; uint8_t len; uint8_t buf[8];
                servos.can.readMsgBuf(&id, &len, buf);
                if (id >= 1 && id <= 6) idCnt2[id]++;
            }
        }
        Serial.printf("{\"xdiag\":\"spam_after_query\",\"per_id\":[%d,%d,%d,%d,%d,%d]}\n",
                      idCnt2[1],idCnt2[2],idCnt2[3],idCnt2[4],idCnt2[5],idCnt2[6]);
        Serial.println("{\"xdiag\":\"done\"}");
        posEnabled = savedPos; holdMode = savedHold;
    }
}

void loop() {
    handleSerial();

    // auto-return：每次 loop 迭代都排空（高頻），避免 streaming 幀塞爆 2-buffer
    if (autoReturnMode) servos.drainInto(latestRaw);

    // 控制迴圈閘門（micros，週期可調）
    static uint32_t lastReadUs = 0;
    static uint32_t ctrlCycles = 0;
    uint32_t nowUs = micros();
    if (nowUs - lastReadUs < loopPeriodUs) return;
    uint32_t elapsed = (nowUs - lastReadUs) / 1000;  // ms
    lastReadUs = nowUs;
    ctrlCycles++;

    // 真實取樣間隔（秒），餵控制器的時間導數/平滑項。clamp 到標稱的 [0.5×, 3×]：
    // 過短 → vel=noise/dt 把 D 項炸成高頻放大器；stall 後一個巨大 Δ 配正常 dt 會 spike。
    float nominalDt = loopPeriodUs / 1e6f;
    float dt = fmaxf(0.5f * nominalDt, fminf(3.0f * nominalDt, (float)elapsed / 1000.0f));

    int64_t raw[NUM_MOTORS];
    int ok = 0;
    uint32_t canReadUs = 0;
    if (autoReturnMode) {
        // 連續排空已在 loop 頂端進行，這裡只取最新快照（感測延遲 ≈ 0）
        servos.drainInto(latestRaw);
        for (int i = 0; i < NUM_MOTORS; i++) {
            raw[i] = latestRaw[i];
            if (raw[i] != INT64_MIN) ok++;
        }
    } else {
        servos.flushReceiveBuffer();
        uint32_t _canT0 = micros();
        ok = servos.readAllRawEncoders(raw);
        canReadUs = micros() - _canT0;   // 讀 6 顆編碼器的純 CAN 來回時間（頻寬指紋核心）
    }
    enc.updateAngles(raw);

    float motorTargets[NUM_MOTORS] = {0};
    int32_t coords[NUM_MOTORS] = {0};

    if (posEnabled) {
        bool controlOk = false;

        if (holdMode) {
            // ===== HOLD：直送 snapshot，跳過所有 PD/IK/平滑 =====
            // 姿態移動（G）：平滑過渡 holdAngles → holdTarget（smoothstep 加減速，避免階躍衝擊）
            if (holdMoveMs > 0) {
                float t = (float)(millis() - holdMoveStart) / holdMoveMs;
                if (t >= 1.0f) {
                    for (int i = 0; i < NUM_MOTORS; i++) holdAngles[i] = holdTarget[i];
                    holdMoveMs = 0;
                } else {
                    float s = t * t * (3.0f - 2.0f * t);   // smoothstep
                    for (int i = 0; i < NUM_MOTORS; i++)
                        holdAngles[i] = holdStart[i] + (holdTarget[i] - holdStart[i]) * s;
                }
            }
            bool bumpActive = (bumpUntilMs != 0 && millis() < bumpUntilMs);
            if (bumpUntilMs != 0 && !bumpActive) {               // 脈衝結束，六軸偏移歸零
                for (int i = 0; i < NUM_MOTORS; i++) bumpDeg[i] = 0;
                bumpUntilMs = 0;
            }
            for (int i = 0; i < NUM_MOTORS; i++) {
                motorTargets[i] = holdAngles[i];
                if (bumpActive) motorTargets[i] += bumpDeg[i];
            }
            float maxAbs = 0;
            for (int i = 0; i < NUM_MOTORS; i++) {
                float e = fabsf(enc.angles[i] - holdAngles[i]);
                if (e > maxAbs) maxAbs = e;
            }
            if (maxAbs > maxHoldErr) maxHoldErr = maxAbs;
            controlOk = true;
        } else if (controlMode == 1) {
            // ===== Task-space PD =====
            controlOk = tsController.update(enc.angles, targetPose, motorTargets, dt);

            // FK 連續失敗 5 次 → 自動降回 joint-space
            if (tsController.fkFailCount >= 5) {
                controlMode = 0;
                smoothedTarget = targetPose;
                for (int i = 0; i < NUM_MOTORS; i++)
                    prevAngles[i] = enc.angles[i];
                adaptiveFirstCycle = true;
                Serial.println("{\"warn\":\"FK fail x5, fallback to joint-space\"}");
            }
        }

        if (!holdMode && controlMode == 0) {
            // ===== Joint-space 備用 =====
            // 平滑時間常數用真實 dt（改 L 時 smoothing 速率才正確）；
            // 注意：下方自適應追蹤的 vel 是 per-cycle Δ（不除 dt）為刻意設計，
            // 該公式本質與迴圈率耦合，未在此正規化（見 audit Gate A confound 說明）。
            smoothPose(smoothedTarget, targetPose, dt);

            IKResult target = inverse_kinematics(smoothedTarget);
            if (!target.valid) {
                posStop();
                Serial.println("{\"error\":\"IK invalid, pos stopped\"}");
            } else {
                if (adaptiveFirstCycle) {
                    for (int i = 0; i < NUM_MOTORS; i++)
                        prevAngles[i] = enc.angles[i];
                    smoothKinetic = 0;
                    adaptiveFirstCycle = false;
                }

                float vel[NUM_MOTORS];
                float kinetic = 0;
                for (int i = 0; i < NUM_MOTORS; i++) {
                    vel[i] = enc.angles[i] - prevAngles[i];
                    kinetic += vel[i] * vel[i];
                }

                smoothKinetic = 0.9f * smoothKinetic + 0.1f * kinetic;
                float gain = fminf(maxGain, 1.0f / (1.0f + trackingMu * smoothKinetic));

                for (int i = 0; i < NUM_MOTORS; i++) {
                    float error = target.angles[i] - enc.angles[i];
                    float dampRatio = (error * vel[i] < 0) ? 1.0f : 0.3f;
                    float damp = trackingKd * vel[i] * dampRatio;
                    motorTargets[i] = enc.angles[i] + gain * error - damp;
                    prevAngles[i] = enc.angles[i];
                }
                controlOk = true;
            }
        }

        // 發送 F5 指令
        if (controlOk) {
            for (int i = 0; i < NUM_MOTORS; i++) {
                coords[i] = enc.angleToCoord(i, motorTargets[i], enableAngle[i]);
                if (abs(coords[i]) > 8192) {
                    posStop();
                    Serial.println("{\"error\":\"coord out of range, pos stopped\"}");
                    controlOk = false;
                    break;
                }
                servos.setAbsoluteCoord(MOTOR_ADDR[i], posSpeed, posAcc, coords[i]);
            }
        }
    }

    // 遙測與控制解耦：每 TELE_PERIOD_MS 才輸出（高速迴圈下全遙測會塞爆 115200 序列埠）
    static uint32_t lastTeleMs = 0;
    uint32_t nowMs = millis();
    if (nowMs - lastTeleMs < TELE_PERIOD_MS) return;
    float lhz = ctrlCycles * 1000.0f / (nowMs - lastTeleMs);  // 實測控制迴圈頻率
    ctrlCycles = 0;
    lastTeleMs = nowMs;

    // JSON 輸出（ef=EFLG, tx=TEC, rx=REC, lhz=實測控制Hz, ar=上報週期/0=輪詢）
    uint8_t eflg = servos.can.getEFLG();
    uint8_t tec  = servos.can.getTEC();
    uint8_t rec  = servos.can.getREC();
    if (eflg & 0xC0) servos.can.clearRXOverflow();  // 清 RX overflow 才能繼續看新事件

    Serial.printf("{\"a\":[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f],"
                  "\"r\":[%lld,%lld,%lld,%lld,%lld,%lld],"
                  "\"ok\":%d,\"z\":%d,\"pos\":%d,\"cm\":%d,"
                  "\"ef\":%u,\"tx\":%u,\"rx\":%u,"
                  "\"t\":%u,\"dt\":%u,\"cus\":%u,\"lhz\":%.0f,\"ar\":%d,"
                  "\"pid\":[%u,%u,%u,%u]",
        enc.angles[0], enc.angles[1], enc.angles[2],
        enc.angles[3], enc.angles[4], enc.angles[5],
        raw[0], raw[1], raw[2], raw[3], raw[4], raw[5],
        ok, enc.zeroed ? 1 : 0, posEnabled ? 1 : 0, controlMode,
        eflg, tec, rec,
        nowMs, elapsed, canReadUs, lhz, autoReturnMode ? arPeriodMs : 0,
        curKp, curKi, curKd, curKv);

    if (posEnabled && holdMode) {
        float herr[NUM_MOTORS];
        for (int i = 0; i < NUM_MOTORS; i++) herr[i] = enc.angles[i] - holdAngles[i];
        Serial.printf(",\"hold\":[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f],"
                      "\"herr\":[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f],"
                      "\"hmax\":%.2f",
            holdAngles[0],holdAngles[1],holdAngles[2],
            holdAngles[3],holdAngles[4],holdAngles[5],
            herr[0],herr[1],herr[2],herr[3],herr[4],herr[5],
            maxHoldErr);
    } else if (posEnabled && controlMode == 1) {
        const Pose& fk = tsController.currentPose;
        const Pose& er = tsController.poseError;
        Serial.printf(",\"fk\":[%.1f,%.1f,%.1f,%.2f,%.2f,%.2f],"
                      "\"err\":[%.1f,%.1f,%.1f,%.2f,%.2f,%.2f],"
                      "\"tgt\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f],"
                      "\"fki\":%d",
            fk.x, fk.y, fk.z, fk.roll, fk.pitch, fk.yaw,
            er.x, er.y, er.z, er.roll, er.pitch, er.yaw,
            motorTargets[0],motorTargets[1],motorTargets[2],
            motorTargets[3],motorTargets[4],motorTargets[5],
            tsController.fk.iterations);
    } else if (posEnabled && controlMode == 0) {
        Serial.printf(",\"tgt\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f],"
                      "\"g\":%.3f",
            motorTargets[0],motorTargets[1],motorTargets[2],
            motorTargets[3],motorTargets[4],motorTargets[5],
            maxGain);
    }

    Serial.println("}");
}
