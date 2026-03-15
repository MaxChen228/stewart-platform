#pragma once
#include <mcp_can.h>
#include <SPI.h>

// SPI 腳位
constexpr uint8_t CAN_CS_PIN  = 5;
constexpr uint8_t CAN_INT_PIN = 17;

// MCP2515 晶振頻率（常見 8MHz，若模組是 16MHz 改 MCP_16MHZ）
constexpr uint8_t CAN_CRYSTAL = MCP_8MHZ;

// 馬達數量與位址
constexpr uint8_t NUM_MOTORS = 6;
constexpr uint8_t MOTOR_ADDR[NUM_MOTORS] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};

// 馬達方向（參考專案中 2/4/6 反向）
constexpr int8_t MOTOR_SIGN[NUM_MOTORS] = {1, -1, 1, -1, 1, -1};

class Servo42D {
public:
    MCP_CAN can;
    bool initialized = false;

    Servo42D() : can(CAN_CS_PIN) {}

    bool begin() {
        if (can.begin(MCP_ANY, CAN_500KBPS, CAN_CRYSTAL) != CAN_OK) return false;
        can.setMode(MCP_NORMAL);
        initialized = true;
        return true;
    }

    // Debug: dump 一次 CAN 回覆的原始 bytes
    bool debugDumped = false;

    // 讀取單顆馬達的編碼器原始值（14-bit, 0~16383）
    // 回傳 -1 表示通訊失敗，-2 表示 CRC 錯誤
    int32_t readEncoderRaw(uint8_t motorId) {
        uint8_t cmd[2];
        cmd[0] = 0x30;
        cmd[1] = (motorId + 0x30) & 0xFF;

        if (can.sendMsgBuf(motorId, 0, 2, cmd) != CAN_OK) return -1;

        uint32_t start = millis();
        while (millis() - start < 10) {
            if (can.checkReceive() == CAN_MSGAVAIL) {
                unsigned long rxId;
                uint8_t rxLen;
                uint8_t rxBuf[8];
                can.readMsgBuf(&rxId, &rxLen, rxBuf);

                if ((rxId & 0x7FF) != motorId) continue;    // 不是這顆馬達
                if (rxBuf[0] != 0x30) continue;             // 不是 encoder 回覆（可能是 F5 回覆）
                if (rxLen < 8) continue;                     // 長度不對

                // Debug: 印出前幾次的原始回覆
                if (!debugDumped) {
                    Serial.printf("{\"debug\":\"id=0x%02lX len=%d data=\"", rxId, rxLen);
                    for (int k = 0; k < rxLen; k++) Serial.printf("%02X ", rxBuf[k]);
                    Serial.println("\"}");
                }

                // CRC 驗證：CRC = (motorId + data[0..6]) & 0xFF
                uint8_t crc = motorId;
                for (int k = 0; k < 7; k++) crc += rxBuf[k];
                if ((crc & 0xFF) != rxBuf[7]) return -2;    // CRC 錯誤

                uint16_t value = ((uint16_t)rxBuf[5] << 8) | rxBuf[6];
                return value & 0x3FFF;
            }
        }
        return -1;
    }

    // 讀取馬達累計坐標值 (0x31) — F5 使用的絕對坐標系
    // 回傳 INT32_MIN 表示失敗
    int32_t readCoordinate(uint8_t motorId) {
        uint8_t cmd[3];
        cmd[0] = 0x31;
        cmd[1] = 0x00; // 校正後的值
        cmd[2] = (motorId + 0x31 + 0x00) & 0xFF;

        if (can.sendMsgBuf(motorId, 0, 3, cmd) != CAN_OK) return INT32_MIN;

        uint32_t start = millis();
        while (millis() - start < 20) {
            if (can.checkReceive() == CAN_MSGAVAIL) {
                unsigned long rxId;
                uint8_t rxLen;
                uint8_t rxBuf[8];
                can.readMsgBuf(&rxId, &rxLen, rxBuf);
                if ((rxId & 0x7FF) == motorId && rxBuf[0] == 0x31 && rxLen >= 7) {
                    // 回應: [0x31, carry_b3..b0, val_hi, val_lo, CRC]
                    int32_t carry = ((int32_t)(int8_t)rxBuf[1] << 24) |
                                    ((int32_t)rxBuf[2] << 16) |
                                    ((int32_t)rxBuf[3] << 8) |
                                    rxBuf[4];
                    uint16_t val = ((uint16_t)rxBuf[5] << 8) | rxBuf[6];
                    return carry * 16384 + (int32_t)(val & 0x3FFF);
                }
            }
        }
        return INT32_MIN;
    }

    // 清空 CAN 接收緩衝區（丟棄所有待讀訊息）
    void flushReceiveBuffer() {
        unsigned long rxId;
        uint8_t rxLen;
        uint8_t rxBuf[8];
        while (can.checkReceive() == CAN_MSGAVAIL) {
            can.readMsgBuf(&rxId, &rxLen, rxBuf);
        }
    }

    // 讀取所有馬達編碼器原始值（0x30，保留供相容）
    int readAllEncoders(int32_t rawValues[NUM_MOTORS]) {
        int ok = 0;
        for (int i = 0; i < NUM_MOTORS; i++) {
            rawValues[i] = readEncoderRaw(MOTOR_ADDR[i]);
            if (rawValues[i] >= 0) ok++;
        }
        debugDumped = true;
        return ok;
    }

    // ===== 0x35: 讀取 RAW 累計編碼器值（不受 0x92 影響）=====
    // 回傳 INT64_MIN 表示失敗
    int64_t readRawEncoderValue(uint8_t motorId) {
        uint8_t cmd[2];
        cmd[0] = 0x35;
        cmd[1] = (motorId + 0x35) & 0xFF;

        if (can.sendMsgBuf(motorId, 0, 2, cmd) != CAN_OK) return INT64_MIN;

        uint32_t start = millis();
        while (millis() - start < 10) {
            if (can.checkReceive() == CAN_MSGAVAIL) {
                unsigned long rxId;
                uint8_t rxLen;
                uint8_t rxBuf[8];
                can.readMsgBuf(&rxId, &rxLen, rxBuf);

                if ((rxId & 0x7FF) != motorId) continue;
                if (rxBuf[0] != 0x35) continue;
                if (rxLen < 8) continue;

                // CRC 驗證
                uint8_t crc = motorId;
                for (int k = 0; k < 7; k++) crc += rxBuf[k];
                if ((crc & 0xFF) != rxBuf[7]) continue;

                // 解析 int48 大端序，符號擴展
                int64_t val = (int64_t)(int8_t)rxBuf[1];
                val = (val << 8) | rxBuf[2];
                val = (val << 8) | rxBuf[3];
                val = (val << 8) | rxBuf[4];
                val = (val << 8) | rxBuf[5];
                val = (val << 8) | rxBuf[6];
                return val;
            }
        }
        return INT64_MIN;
    }

    int readAllRawEncoders(int64_t rawValues[NUM_MOTORS]) {
        int ok = 0;
        for (int i = 0; i < NUM_MOTORS; i++) {
            rawValues[i] = readRawEncoderValue(MOTOR_ADDR[i]);
            if (rawValues[i] != INT64_MIN) ok++;
        }
        return ok;
    }

    // 使能/禁用馬達 (0xF3)
    void setEnable(uint8_t motorId, bool enable) {
        uint8_t cmd[3];
        cmd[0] = 0xF3;
        cmd[1] = enable ? 0x01 : 0x00;
        cmd[2] = (motorId + cmd[0] + cmd[1]) & 0xFF;
        can.sendMsgBuf(motorId, 0, 3, cmd);
    }

    // 禁用所有馬達
    void disableAll() {
        for (int i = 0; i < NUM_MOTORS; i++) {
            setEnable(MOTOR_ADDR[i], false);
        }
    }

    // 速度控制 (0xF6)
    // speed: 0-3000 RPM, dir: 0=CCW 1=CW, acc: 0-255 (0=instant)
    void setSpeed(uint8_t motorId, uint16_t speed, uint8_t dir, uint8_t acc) {
        if (speed > 3000) speed = 3000;
        uint8_t cmd[5];
        cmd[0] = 0xF6;
        cmd[1] = (dir ? 0x80 : 0x00) | ((speed >> 8) & 0x0F);
        cmd[2] = speed & 0xFF;
        cmd[3] = acc;
        cmd[4] = (motorId + cmd[0] + cmd[1] + cmd[2] + cmd[3]) & 0xFF;
        can.sendMsgBuf(motorId, 0, 5, cmd);
    }

    // 停止所有馬達速度模式
    void stopAllSpeed() {
        for (int i = 0; i < NUM_MOTORS; i++) {
            setSpeed(MOTOR_ADDR[i], 0, 0, 0);
        }
    }

    // 緊急停止 (0xF7)
    void emergencyStop(uint8_t motorId) {
        uint8_t cmd[2];
        cmd[0] = 0xF7;
        cmd[1] = (motorId + 0xF7) & 0xFF;
        can.sendMsgBuf(motorId, 0, 2, cmd);
    }

    void emergencyStopAll() {
        for (int i = 0; i < NUM_MOTORS; i++) {
            emergencyStop(MOTOR_ADDR[i]);
        }
    }

    // 設定馬達零點 (0x92) — 將當前位置設為坐標原點
    bool setZeroPoint(uint8_t motorId) {
        uint8_t cmd[2];
        cmd[0] = 0x92;
        cmd[1] = (motorId + 0x92) & 0xFF;
        can.sendMsgBuf(motorId, 0, 2, cmd);

        // 等待回覆
        uint32_t start = millis();
        while (millis() - start < 50) {
            if (can.checkReceive() == CAN_MSGAVAIL) {
                unsigned long rxId;
                uint8_t rxLen;
                uint8_t rxBuf[8];
                can.readMsgBuf(&rxId, &rxLen, rxBuf);
                if ((rxId & 0x7FF) == motorId && rxBuf[0] == 0x92) {
                    return rxBuf[1] == 1;
                }
            }
        }
        return false;
    }

    // 絕對坐標位置指令 (0xF5)
    // speed: 0-3000 RPM（最大移動速度）, acc: 0-255, coord: int24_t 絕對坐標值 (16384 counts/turn)
    void setAbsoluteCoord(uint8_t motorId, uint16_t speed, uint8_t acc, int32_t coord) {
        if (speed > 3000) speed = 3000;
        if (coord > 8388607) coord = 8388607;
        if (coord < -8388607) coord = -8388607;

        uint8_t cmd[8];
        cmd[0] = 0xF5;
        cmd[1] = (speed >> 8) & 0xFF;
        cmd[2] = speed & 0xFF;
        cmd[3] = acc;
        cmd[4] = (coord >> 16) & 0xFF;
        cmd[5] = (coord >> 8) & 0xFF;
        cmd[6] = coord & 0xFF;
        cmd[7] = motorId;
        for (int i = 0; i < 7; i++) cmd[7] += cmd[i];
        cmd[7] &= 0xFF;
        can.sendMsgBuf(motorId, 0, 8, cmd);
    }

    // 停止絕對坐標運動 (0xF5 speed=0)
    void stopAbsoluteCoord(uint8_t motorId, uint8_t acc = 5) {
        uint8_t cmd[8];
        cmd[0] = 0xF5;
        cmd[1] = 0; cmd[2] = 0; // speed = 0
        cmd[3] = acc;
        cmd[4] = 0; cmd[5] = 0; cmd[6] = 0; // coord = 0
        cmd[7] = motorId;
        for (int i = 0; i < 7; i++) cmd[7] += cmd[i];
        cmd[7] &= 0xFF;
        can.sendMsgBuf(motorId, 0, 8, cmd);
    }

    void stopAllPosition(uint8_t acc = 5) {
        for (int i = 0; i < NUM_MOTORS; i++) {
            stopAbsoluteCoord(MOTOR_ADDR[i], acc);
        }
    }

    // 設定 vFOC 模式 PID 參數 (0x96)
    // Kp, Ki: 位置環比例和積分
    void setVFOC_KpKi(uint8_t motorId, uint16_t kp, uint16_t ki) {
        uint8_t cmd[7];
        cmd[0] = 0x96;
        cmd[1] = 0x00; // CMD = set Kp/Ki
        cmd[2] = (kp >> 8) & 0xFF;
        cmd[3] = kp & 0xFF;
        cmd[4] = (ki >> 8) & 0xFF;
        cmd[5] = ki & 0xFF;
        cmd[6] = motorId;
        for (int i = 0; i < 6; i++) cmd[6] += cmd[i];
        cmd[6] &= 0xFF;
        can.sendMsgBuf(motorId, 0, 7, cmd);
    }

    // Kd, Kv: 位置環微分和速度環增益
    void setVFOC_KdKv(uint8_t motorId, uint16_t kd, uint16_t kv) {
        uint8_t cmd[7];
        cmd[0] = 0x96;
        cmd[1] = 0x01; // CMD = set Kd/Kv
        cmd[2] = (kd >> 8) & 0xFF;
        cmd[3] = kd & 0xFF;
        cmd[4] = (kv >> 8) & 0xFF;
        cmd[5] = kv & 0xFF;
        cmd[6] = motorId;
        for (int i = 0; i < 6; i++) cmd[6] += cmd[i];
        cmd[6] &= 0xFF;
        can.sendMsgBuf(motorId, 0, 7, cmd);
    }

    // ===== F4 相對坐標位置指令 =====
    // relCoord: 相對當前位置的增量 (int24, 16384 counts/turn)
    void setRelativeCoord(uint8_t motorId, uint16_t speed, uint8_t acc, int32_t relCoord) {
        if (speed > 3000) speed = 3000;
        if (relCoord > 8388607) relCoord = 8388607;
        if (relCoord < -8388607) relCoord = -8388607;

        uint8_t cmd[8];
        cmd[0] = 0xF4;
        cmd[1] = (speed >> 8) & 0xFF;
        cmd[2] = speed & 0xFF;
        cmd[3] = acc;
        cmd[4] = (relCoord >> 16) & 0xFF;
        cmd[5] = (relCoord >> 8) & 0xFF;
        cmd[6] = relCoord & 0xFF;
        cmd[7] = motorId;
        for (int i = 0; i < 7; i++) cmd[7] += cmd[i];
        cmd[7] &= 0xFF;
        can.sendMsgBuf(motorId, 0, 8, cmd);
    }

    // ===== 多馬達同步 =====
    // 啟用同步模式：馬達收到 F4/F5 後緩衝但不執行，等 trigger
    void enableSync(bool enable = true) {
        uint8_t cmd[3];
        cmd[0] = 0x4A;
        cmd[1] = enable ? 0x01 : 0x00;
        cmd[2] = (cmd[0] + cmd[1]) & 0xFF; // broadcast CRC (ID=0)
        can.sendMsgBuf(0x00, 0, 3, cmd);   // broadcast
    }

    // 觸發同步執行（建議連發 2-3 次，間隔 1ms）
    void triggerSync() {
        uint8_t cmd[2];
        cmd[0] = 0x4B;
        cmd[1] = 0x4B; // broadcast CRC (ID=0, 0x00+0x4B=0x4B)
        can.sendMsgBuf(0x00, 0, 2, cmd);
    }
};
