#pragma once
#include <SPI.h>
#include <mcp2515.h>

// SPI 腳位
constexpr uint8_t CAN_CS_PIN  = 5;
constexpr uint8_t CAN_INT_PIN = 17;

// ===== Compat shim: 把 autowp/arduino-mcp2515 包成原 coryjfowler/mcp_can API =====
// 原因：coryjfowler 在 ESP32 arduino-core 2.x 有 SPI lifecycle bug（spi_t* 損壞）
// 換 lib 後保持上層 sendMsgBuf / checkReceive / readMsgBuf 介面不動
#define CAN_OK        0
#define CAN_FAIL      1
#define CAN_MSGAVAIL  3
#define CAN_NOMSG     1

class MCP_CAN {
    MCP2515 mcp;
    can_frame cached;
    bool hasCached = false;
public:
    MCP_CAN(uint8_t cs) : mcp(cs) {}

    bool begin() {
        mcp.reset();
        if (mcp.setBitrate(CAN_500KBPS, MCP_8MHZ) != MCP2515::ERROR_OK) return false;
        // accept all motor IDs 1-6（生產用）
        {
            uint8_t buf[4];
            encodeStdId(0x7F8, buf);
            for (int i = 0; i < 4; i++) rawWriteReg(0x20 + i, buf[i]);
            for (int i = 0; i < 4; i++) rawWriteReg(0x24 + i, buf[i]);
            encodeStdId(0x000, buf);
            uint8_t rxfBase[6] = {0x00, 0x04, 0x08, 0x10, 0x14, 0x18};
            for (int f = 0; f < 6; f++)
                for (int i = 0; i < 4; i++) rawWriteReg(rxfBase[f] + i, buf[i]);
        }
        if (mcp.setNormalMode() != MCP2515::ERROR_OK) return false;
        return true;
    }

    uint8_t sendMsgBuf(uint32_t id, uint8_t /*ext*/, uint8_t len, const uint8_t* buf) {
        can_frame f;
        f.can_id  = id & 0x7FF;  // standard 11-bit
        f.can_dlc = len > 8 ? 8 : len;
        memcpy(f.data, buf, f.can_dlc);
        return (mcp.sendMessage(&f) == MCP2515::ERROR_OK) ? CAN_OK : CAN_FAIL;
    }

    uint8_t checkReceive() {
        if (hasCached) return CAN_MSGAVAIL;
        if (mcp.readMessage(&cached) == MCP2515::ERROR_OK) {
            hasCached = true;
            return CAN_MSGAVAIL;
        }
        return CAN_NOMSG;
    }

    uint8_t readMsgBuf(unsigned long* id, uint8_t* len, uint8_t* buf) {
        if (!hasCached) {
            if (mcp.readMessage(&cached) != MCP2515::ERROR_OK) return CAN_FAIL;
        }
        *id  = cached.can_id;
        *len = cached.can_dlc;
        memcpy(buf, cached.data, cached.can_dlc);
        hasCached = false;
        return CAN_OK;
    }

    // ===== 錯誤計數（用來分離硬體 vs 軟體問題）=====
    uint8_t getEFLG()    { return mcp.getErrorFlags(); }
    uint8_t getTEC()     { return mcp.errorCountTX(); }
    uint8_t getREC()     { return mcp.errorCountRX(); }
    // 清掉 RX overflow bits（讀完後手動 clear）
    void clearRXOverflow() { mcp.clearRXnOVRFlags(); }

    // ===== 硬體 ID 過濾 =====
    // 只接受指定 ID 的 frame，其他都被 MCP2515 硬體丟棄
    bool setHardwareFilterToId(uint32_t id) {
        if (mcp.setFilterMask(MCP2515::MASK0, false, 0x7FF) != MCP2515::ERROR_OK) return false;
        if (mcp.setFilterMask(MCP2515::MASK1, false, 0x7FF) != MCP2515::ERROR_OK) return false;
        if (mcp.setFilter(MCP2515::RXF0, false, id) != MCP2515::ERROR_OK) return false;
        if (mcp.setFilter(MCP2515::RXF1, false, id) != MCP2515::ERROR_OK) return false;
        if (mcp.setNormalMode() != MCP2515::ERROR_OK) return false;
        hasCached = false;
        return true;
    }
    // 還原成「全收」
    bool setHardwareFilterAcceptAll() {
        if (mcp.setFilterMask(MCP2515::MASK0, false, 0) != MCP2515::ERROR_OK) return false;
        if (mcp.setFilterMask(MCP2515::MASK1, false, 0) != MCP2515::ERROR_OK) return false;
        if (mcp.setNormalMode() != MCP2515::ERROR_OK) return false;
        hasCached = false;
        return true;
    }

    // ===== 完全 passive 模式：只看 bus，不 ACK 任何 frame =====
    // 用來判斷 spam 是不是因為 ACK 沒到位才觸發 retry
    bool setListenOnly() {
        if (mcp.setListenOnlyMode() != MCP2515::ERROR_OK) return false;
        hasCached = false;
        return true;
    }
    bool setNormal() {
        if (mcp.setNormalMode() != MCP2515::ERROR_OK) return false;
        hasCached = false;
        return true;
    }

    // 直接 SPI 讀 MCP2515 register
    uint8_t rawReadReg(uint8_t addr) {
        SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
        digitalWrite(CAN_CS_PIN, LOW);
        SPI.transfer(0x03);
        SPI.transfer(addr);
        uint8_t v = SPI.transfer(0x00);
        digitalWrite(CAN_CS_PIN, HIGH);
        SPI.endTransaction();
        return v;
    }
    // 直接 SPI 寫 MCP2515 register
    void rawWriteReg(uint8_t addr, uint8_t val) {
        SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
        digitalWrite(CAN_CS_PIN, LOW);
        SPI.transfer(0x02);   // WRITE instruction
        SPI.transfer(addr);
        SPI.transfer(val);
        digitalWrite(CAN_CS_PIN, HIGH);
        SPI.endTransaction();
    }
    // 切 mode via raw SPI — 直接寫絕對值，不保留 ABAT/OSM
    bool rawSetMode(uint8_t mode_bits) {
        rawWriteReg(0x0F, mode_bits);  // CANCTRL = mode only, clear ABAT/OSM/CLKPRE
        uint32_t t0 = millis();
        while (millis() - t0 < 10) {
            uint8_t st = rawReadReg(0x0E) & 0xE0;
            if (st == mode_bits) return true;
        }
        return false;
    }
    // ===== 純 raw SPI 設 filter，繞過 autowp =====
    // 把 11-bit standard ID 寫進 4-byte buf
    static void encodeStdId(uint32_t id, uint8_t* buf) {
        buf[0] = (uint8_t)(id >> 3);          // SIDH
        buf[1] = (uint8_t)((id & 0x07) << 5); // SIDL (bit 3 EXIDE=0)
        buf[2] = 0;                            // EID8
        buf[3] = 0;                            // EID0
    }
    // 回傳 diagnostic: bit0=mode_to_config_ok, bit1=mode_to_normal_ok, hi=rxm0_in_config
    uint32_t rawSetHardwareFilterDiag(uint32_t targetId) {
        uint32_t diag = 0;
        if (rawSetMode(0x80)) diag |= 1;
        uint8_t buf[4];
        encodeStdId(0x7FF, buf);
        for (int i = 0; i < 4; i++) rawWriteReg(0x20 + i, buf[i]);
        for (int i = 0; i < 4; i++) rawWriteReg(0x24 + i, buf[i]);
        encodeStdId(targetId, buf);
        uint8_t rxfBase[6] = {0x00, 0x04, 0x08, 0x10, 0x14, 0x18};
        for (int f = 0; f < 6; f++)
            for (int i = 0; i < 4; i++) rawWriteReg(rxfBase[f] + i, buf[i]);
        // 在 config mode 讀回確認寫入
        uint8_t rxm0_c = rawReadReg(0x20);
        uint8_t rxf0_c = rawReadReg(0x00);
        uint8_t canctrl_before = rawReadReg(0x0F);
        if (rawSetMode(0x00)) diag |= 2;
        uint8_t rxm0_n = rawReadReg(0x20);
        // 再回 config 看 register 是否還在
        rawSetMode(0x80);
        uint8_t rxm0_c2 = rawReadReg(0x20);
        uint8_t rxb0ctrl = rawReadReg(0x60);
        rawSetMode(0x00);
        hasCached = false;
        diag |= ((uint32_t)rxm0_c << 8) | ((uint32_t)rxm0_n << 16) | ((uint32_t)rxm0_c2 << 24);
        Serial.printf("{\"deep_diag\":{\"rxb0ctrl_in_config\":\"0x%02X\"}}\n", rxb0ctrl);
        return diag;
    }
    bool rawSetHardwareFilter(uint32_t targetId) {
        return rawSetHardwareFilterDiag(targetId) & 3;
    }
    bool rawSetFilterAcceptAll() {
        if (!rawSetMode(0x80)) return false;
        for (int i = 0; i < 4; i++) rawWriteReg(0x20 + i, 0);  // RXM0 = 0 (don't care)
        for (int i = 0; i < 4; i++) rawWriteReg(0x24 + i, 0);  // RXM1 = 0
        if (!rawSetMode(0x00)) return false;
        hasCached = false;
        return true;
    }
};

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
        if (!can.begin()) return false;
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
    int64_t readRawEncoderValue(uint8_t motorId, uint32_t timeoutMs = 10) {
        uint8_t cmd[2];
        cmd[0] = 0x35;
        cmd[1] = (motorId + 0x35) & 0xFF;

        if (can.sendMsgBuf(motorId, 0, 2, cmd) != CAN_OK) return INT64_MIN;

        uint32_t start = millis();
        while (millis() - start < timeoutMs) {
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

    // 非阻塞排空：把當前 RX buffer 內所有 0x35 回覆配對解析進 rawValues，回傳新收數。
    int drainEncoderReplies(int64_t rawValues[NUM_MOTORS]) {
        int n = 0;
        while (can.checkReceive() == CAN_MSGAVAIL) {
            unsigned long rxId; uint8_t rxLen; uint8_t rxBuf[8];
            can.readMsgBuf(&rxId, &rxLen, rxBuf);
            uint16_t id = rxId & 0x7FF;
            if (rxBuf[0] != 0x35 || rxLen < 8) continue;

            int idx = -1;
            for (int i = 0; i < NUM_MOTORS; i++)
                if (MOTOR_ADDR[i] == id) { idx = i; break; }
            if (idx < 0 || rawValues[idx] != INT64_MIN) continue;  // 未知 id / 已收過

            uint8_t crc = id;
            for (int k = 0; k < 7; k++) crc += rxBuf[k];
            if ((crc & 0xFF) != rxBuf[7]) continue;

            int64_t val = (int64_t)(int8_t)rxBuf[1];
            val = (val << 8) | rxBuf[2];
            val = (val << 8) | rxBuf[3];
            val = (val << 8) | rxBuf[4];
            val = (val << 8) | rxBuf[5];
            val = (val << 8) | rxBuf[6];
            rawValues[idx] = val;
            n++;
        }
        return n;
    }

    // 設定馬達定時主動上報唯讀參數（0x01）。code=0x35 位置；periodMs=0 停用。
    bool setAutoReturn(uint8_t motorId, uint8_t code, uint16_t periodMs) {
        uint8_t cmd[5];
        cmd[0] = 0x01;
        cmd[1] = code;
        cmd[2] = (periodMs >> 8) & 0xFF;
        cmd[3] = periodMs & 0xFF;
        uint8_t crc = motorId;
        for (int k = 0; k < 4; k++) crc += cmd[k];
        cmd[4] = crc & 0xFF;
        return can.sendMsgBuf(motorId, 0, 5, cmd) == CAN_OK;
    }

    // 連續排空：把 RX buffer 內所有 0x35 上報幀「覆蓋寫入」latest[]（永遠保留最新值）。
    // 與 drainEncoderReplies 差別：不跳過已收的，總是更新。供 auto-return 高頻呼叫。
    int drainInto(int64_t latest[NUM_MOTORS]) {
        int n = 0;
        while (can.checkReceive() == CAN_MSGAVAIL) {
            unsigned long rxId; uint8_t rxLen; uint8_t rxBuf[8];
            can.readMsgBuf(&rxId, &rxLen, rxBuf);
            uint16_t id = rxId & 0x7FF;
            if (rxBuf[0] != 0x35 || rxLen < 8) continue;
            int idx = -1;
            for (int i = 0; i < NUM_MOTORS; i++)
                if (MOTOR_ADDR[i] == id) { idx = i; break; }
            if (idx < 0) continue;
            uint8_t crc = id;
            for (int k = 0; k < 7; k++) crc += rxBuf[k];
            if ((crc & 0xFF) != rxBuf[7]) continue;
            int64_t val = (int64_t)(int8_t)rxBuf[1];
            val = (val << 8) | rxBuf[2];
            val = (val << 8) | rxBuf[3];
            val = (val << 8) | rxBuf[4];
            val = (val << 8) | rxBuf[5];
            val = (val << 8) | rxBuf[6];
            latest[idx] = val;
            n++;
        }
        return n;
    }

    // Pipelining：送一個 0x35 查詢就立刻排空已到回覆，避免 2-buffer MCP2515
    // 在 send 期間溢位（naive「先全送再收」實測 18% 掉幀、撞 timeout）。
    int readAllRawEncoders(int64_t rawValues[NUM_MOTORS]) {
        for (int i = 0; i < NUM_MOTORS; i++) rawValues[i] = INT64_MIN;
        int got = 0;

        // 每 cycle 輪轉查詢起始順序：最後查詢的那顆回覆最易撞滿 2-buffer MCP2515 而掉，
        // 輪轉讓 staleness 公平分攤到 6 顆，不讓固定某顆一直變舊（實測掉幀隨位置走非馬達身分）。
        static uint8_t rot = 0;
        rot = (rot + 1) % NUM_MOTORS;
        for (int k = 0; k < NUM_MOTORS; k++) {
            int i = (rot + k) % NUM_MOTORS;
            uint8_t cmd[2] = { 0x35, (uint8_t)((MOTOR_ADDR[i] + 0x35) & 0xFF) };
            can.sendMsgBuf(MOTOR_ADDR[i], 0, 2, cmd);
            got += drainEncoderReplies(rawValues);   // 每送一個就清一次
        }

        uint32_t start = millis();
        while (got < NUM_MOTORS && (millis() - start) < 8) {
            got += drainEncoderReplies(rawValues);
        }
        return got;
    }

    // 使能/禁用馬達 (0xF3)
    // 回傳 true 表示馬達實際 ACK 0xF3 成功；最多 retry 3 次
    bool setEnable(uint8_t motorId, bool enable) {
        uint8_t cmd[3];
        cmd[0] = 0xF3;
        cmd[1] = enable ? 0x01 : 0x00;
        cmd[2] = (motorId + cmd[0] + cmd[1]) & 0xFF;
        for (int r = 0; r < 3; r++) {
            flushReceiveBuffer();
            if (can.sendMsgBuf(motorId, 0, 3, cmd) != CAN_OK) {
                delay(5);
                continue;
            }
            // 等馬達回 0xF3 ACK（status 1=success, 0=fail）
            uint32_t start = millis();
            while (millis() - start < 30) {
                if (can.checkReceive() == CAN_MSGAVAIL) {
                    unsigned long rxId; uint8_t rxLen; uint8_t rxBuf[8];
                    can.readMsgBuf(&rxId, &rxLen, rxBuf);
                    if ((rxId & 0x7FF) == motorId && rxLen >= 2 && rxBuf[0] == 0xF3) {
                        return rxBuf[1] == 1;
                    }
                }
            }
            delay(5);
        }
        return false;
    }

    // 禁用所有馬達：每顆之間 delay + 失敗重試，避免 MCP2515 TX buffer 滿被丟
    void disableAll() {
        for (int i = 0; i < NUM_MOTORS; i++) {
            bool ok = setEnable(MOTOR_ADDR[i], false);
            if (!ok) {
                Serial.printf("{\"warn\":\"disable M%d send failed\"}\n", i+1);
            }
            delay(8);          // 給 MCP2515 TX buffer 排空 + 馬達回 ACK 時間
            flushReceiveBuffer();
        }
    }

    // 使能所有馬達：回傳 per-motor success mask (bit i = motor i+1)
    // 同樣每顆之間 delay + 失敗重試，避免 TX buffer 滿被丟
    uint8_t enableAll() {
        uint8_t mask = 0;
        for (int i = 0; i < NUM_MOTORS; i++) {
            bool ok = setEnable(MOTOR_ADDR[i], true);
            if (ok) mask |= (1 << i);
            else Serial.printf("{\"warn\":\"enable M%d send failed\"}\n", i+1);
            delay(8);
            flushReceiveBuffer();
        }
        return mask;
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
            delay(5);
            flushReceiveBuffer();
        }
    }

    // 設定 vFOC 模式 PID 參數 (0x96)
    // Kp, Ki: 位置環比例和積分
    bool setVFOC_KpKi(uint8_t motorId, uint16_t kp, uint16_t ki) {
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
        for (int r = 0; r < 3; r++) {
            if (can.sendMsgBuf(motorId, 0, 7, cmd) == CAN_OK) return true;
            delay(5); flushReceiveBuffer();
        }
        return false;
    }

    // 0x83 設工作電流（不存 flash 版本，DLC=5）
    // current: uint16 mA, 42D max 3000
    bool setWorkingCurrent(uint8_t motorId, uint16_t current) {
        if (current > 3000) current = 3000;
        uint8_t cmd[5];
        cmd[0] = 0x83;
        cmd[1] = (current >> 8) & 0xFF;
        cmd[2] = current & 0xFF;
        cmd[3] = 0x00; // no save
        cmd[4] = (motorId + cmd[0] + cmd[1] + cmd[2] + cmd[3]) & 0xFF;
        for (int r = 0; r < 3; r++) {
            if (can.sendMsgBuf(motorId, 0, 5, cmd) == CAN_OK) return true;
            delay(5); flushReceiveBuffer();
        }
        return false;
    }

    // 0x8C 設回應模式：XX=1 YY=0 → 只回 start/fail 不回 completion
    // 減少 MCP2515 RX buffer 壓力（每 cycle 6 顆 F5 不會塞爆）
    void setResponseMode(uint8_t motorId, uint8_t xx, uint8_t yy) {
        uint8_t cmd[4];
        cmd[0] = 0x8C;
        cmd[1] = xx;
        cmd[2] = yy;
        cmd[3] = (motorId + cmd[0] + cmd[1] + cmd[2]) & 0xFF;
        can.sendMsgBuf(motorId, 0, 4, cmd);
    }

    // Kd, Kv: 位置環微分和速度環增益
    bool setVFOC_KdKv(uint8_t motorId, uint16_t kd, uint16_t kv) {
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
        for (int r = 0; r < 3; r++) {
            if (can.sendMsgBuf(motorId, 0, 7, cmd) == CAN_OK) return true;
            delay(5); flushReceiveBuffer();
        }
        return false;
    }
};
