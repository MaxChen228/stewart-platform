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
    // 回傳 -1 表示通訊失敗
    int32_t readEncoderRaw(uint8_t motorId) {
        uint8_t cmd[2];
        cmd[0] = 0x30;
        cmd[1] = (motorId + 0x30) & 0xFF;

        if (can.sendMsgBuf(motorId, 0, 2, cmd) != CAN_OK) return -1;

        // 等待回覆
        uint32_t start = millis();
        while (millis() - start < 10) {
            if (can.checkReceive() == CAN_MSGAVAIL) {
                unsigned long rxId;
                uint8_t rxLen;
                uint8_t rxBuf[8];
                can.readMsgBuf(&rxId, &rxLen, rxBuf);

                if ((rxId & 0x7FF) == motorId) {
                    // Debug: 印出前幾次的原始回覆
                    if (!debugDumped) {
                        Serial.printf("{\"debug\":\"id=0x%02lX len=%d data=\"", rxId, rxLen);
                        for (int k = 0; k < rxLen; k++) Serial.printf("%02X ", rxBuf[k]);
                        Serial.println("\"}");
                    }

                    if (rxLen >= 7) {
                        // 格式: [0x30, carry×4, val_hi, val_lo, CRC] = 8 bytes
                        uint16_t value = ((uint16_t)rxBuf[5] << 8) | rxBuf[6];
                        return value & 0x3FFF;
                    }
                }
            }
        }
        return -1;
    }

    // 讀取所有馬達編碼器原始值
    // 回傳成功讀取的馬達數
    int readAllEncoders(int32_t rawValues[NUM_MOTORS]) {
        int ok = 0;
        for (int i = 0; i < NUM_MOTORS; i++) {
            rawValues[i] = readEncoderRaw(MOTOR_ADDR[i]);
            if (rawValues[i] >= 0) ok++;
        }
        debugDumped = true; // 只 dump 第一輪
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
};
