// Standalone TWAI + SN65HVD230(VP230) bring-up probe — READ-ONLY, no motion.
//
// 目的：驗 ESP32 原生 TWAI + VP230 收發器 + 整條 500k CAN bus 是否通。
// 只對馬達發 0x30「讀編碼器」（唯讀，不驅動、不移動），收回覆即證通。
//
// 燒錄：  pio run -e twai_probe -t upload
// 監看：  pio device monitor -e twai_probe -b 460800
// 還原主韌體： npm run upload
//
// 複用既有 TWAI 後端（TX=GPIO21 / RX=GPIO22 / 500k，見 can_twai_compat.h）。
#include "can_twai_compat.h"

static TwaiCanCompat can(0);
static const uint8_t ADDR[] = {1, 2, 3, 4, 5, 6};

// 對單一位址發 0x30，最多等 25ms 收匹配回覆並驗 CRC。
static bool probeOne(uint8_t id) {
    uint8_t cmd[2] = { 0x30, (uint8_t)((id + 0x30) & 0xFF) };  // CRC = (id + 0x30) & 0xFF
    if (can.sendMsgBuf(id, 0, 2, cmd) != TWAI_CAN_OK) {
        Serial.printf("  M0x%02X  TX FAIL (送不出，bus-off/接線?)\n", id);
        return false;
    }
    uint32_t t0 = millis();
    while (millis() - t0 < 25) {
        if (can.checkReceive() != TWAI_CAN_MSGAVAIL) continue;
        unsigned long rid; uint8_t len, rx[8];
        if (can.readMsgBuf(&rid, &len, rx) != TWAI_CAN_OK) continue;
        if ((rid & 0x7FF) != id || len < 8 || rx[0] != 0x30) continue;   // 非本顆 encoder 回覆
        uint8_t crc = id; for (int k = 0; k < 7; k++) crc += rx[k];
        bool ok = ((crc & 0xFF) == rx[7]);
        Serial.printf("  M0x%02X  REPLY  crc=%-3s  raw:", id, ok ? "OK" : "BAD");
        for (int k = 0; k < len; k++) Serial.printf(" %02X", rx[k]);
        Serial.println();
        return ok;
    }
    Serial.printf("  M0x%02X  --- no reply\n", id);
    return false;
}

void setup() {
    Serial.begin(460800);
    delay(400);
    Serial.println("\n=== TWAI probe: VP230, TX=21 RX=22, 500k, READ-ONLY (0x30) ===");
    if (!can.begin(500000)) {
        Serial.println("!! TWAI begin FAILED — driver install/start 失敗，查 3V3 供電/腳位。");
        return;
    }
    Serial.println("TWAI up. 掃描 0x01..0x06 ...（每秒一輪）");
}

void loop() {
    Serial.println("--- scan ---");
    int hit = 0;
    for (uint8_t a : ADDR) if (probeOne(a)) hit++;
    Serial.printf("== %d/6 replied · EFLG=0x%02X TEC=%u REC=%u txFail=%u rxDrop=%u\n\n",
                  hit, can.getEFLG(), can.getTEC(), can.getREC(),
                  can.getTxFailCount(), can.getRxDropCount());
    delay(1000);
}
