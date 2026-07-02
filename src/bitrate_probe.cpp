// Standalone 1M-bitrate migration probe — M5 白老鼠實驗（2026-07-02）。
//
// 實驗問題：
//   Q1  0x8A(set CAN bitrate) 即刻生效還是 power-cycle 生效？
//   Q2  VP230(SN65HVD230) + 現有接線在 1M 的類比裕度（TEC/REC/txFail 統計）？
//   Q3  混速匯流排過渡動力學：500k 旁觀馬達砸 1M 幀多久後 error-passive 閉嘴？
//
// Serial REPL（460800，換行結尾）：
//   B500 / B1M      — ESP32 TWAI 重裝到 500k / 1M
//   S               — 掃描 0x01..0x06（0x30 唯讀）
//   R <id> <n>      — 對單顆連讀 n 次（5ms 間隔），報 ok/fail + TEC/REC/txFail 前後值
//   A <id> <xx> <yy>— 0x8C 設回覆模式（開 ack 才看得到 0x8A 的執行回報）
//   T <id> <code>   — 0x8A 設馬達 CAN bitrate（2=500K, 3=1M）
//   W <id> <n>      — 暖機：對 id 連發 n 筆 0x30 不等回覆（把旁觀節點灌進 error-passive）
//   Q               — 印計數器
// 任何 TX 指令後監聽 150ms，收到的幀全部原樣印出（抓 ack）。
//
// 燒錄：  pio run -e bitrate_probe -t upload   （先 curl -X POST :3000/api/release）
// 還原主韌體： npm run upload
#include "can_twai_compat.h"

static TwaiCanCompat can(0);
static uint32_t curBps = 500000;

static void printStat(const char* tag) {
    Serial.printf("{\"stat\":\"%s\",\"bps\":%u,\"ef\":\"0x%02X\",\"tec\":%u,\"rec\":%u,\"txFail\":%u,\"rxDrop\":%u}\n",
                  tag, curBps, can.getEFLG(), can.getTEC(), can.getREC(),
                  can.getTxFailCount(), can.getRxDropCount());
}

// 監聽 ms 毫秒，印出所有收到的幀（原樣 hex）
static int drainPrint(uint32_t ms, const char* tag) {
    uint32_t t0 = millis();
    int n = 0;
    while (millis() - t0 < ms) {
        if (can.checkReceive() != TWAI_CAN_MSGAVAIL) continue;
        unsigned long rid; uint8_t len, rx[8];
        if (can.readMsgBuf(&rid, &len, rx) != TWAI_CAN_OK) continue;
        Serial.printf("{\"rx\":\"%s\",\"id\":\"0x%02lX\",\"len\":%u,\"data\":\"", tag, rid & 0x7FF, len);
        for (int k = 0; k < len; k++) Serial.printf("%02X%s", rx[k], k + 1 < len ? " " : "");
        Serial.println("\"}");
        n++;
    }
    return n;
}

// 對單顆發 0x30，等 timeout 收匹配回覆
static bool read30(uint8_t id, uint32_t timeoutMs = 25) {
    uint8_t cmd[2] = { 0x30, (uint8_t)((id + 0x30) & 0xFF) };
    if (can.sendMsgBuf(id, 0, 2, cmd) != TWAI_CAN_OK) return false;
    uint32_t t0 = millis();
    while (millis() - t0 < timeoutMs) {
        if (can.checkReceive() != TWAI_CAN_MSGAVAIL) continue;
        unsigned long rid; uint8_t len, rx[8];
        if (can.readMsgBuf(&rid, &len, rx) != TWAI_CAN_OK) continue;
        if ((rid & 0x7FF) != id || len < 8 || rx[0] != 0x30) continue;
        uint8_t crc = id; for (int k = 0; k < 7; k++) crc += rx[k];
        return (crc & 0xFF) == rx[7];
    }
    return false;
}

static void doScan() {
    int hit = 0;
    for (uint8_t id = 1; id <= 6; id++) {
        bool ok = read30(id);
        Serial.printf("{\"scan\":\"M%u\",\"reply\":%d}\n", id, ok ? 1 : 0);
        if (ok) hit++;
        delay(5);
    }
    Serial.printf("{\"scanDone\":%d}\n", hit);
    printStat("post-scan");
}

static void handleLine(String s) {
    s.trim();
    if (s == "B500" || s == "B1M") {
        curBps = (s == "B1M") ? 1000000 : 500000;
        bool ok = can.begin(curBps);
        Serial.printf("{\"bitrate\":%u,\"ok\":%d}\n", curBps, ok ? 1 : 0);
    } else if (s == "S") {
        doScan();
    } else if (s == "Q") {
        printStat("query");
    } else if (s.startsWith("R ")) {
        int id = 0, n = 0;
        if (sscanf(s.c_str(), "R %d %d", &id, &n) == 2 && id >= 1 && id <= 6 && n > 0 && n <= 2000) {
            printStat("pre-R");
            uint32_t fail0 = can.getTxFailCount();
            int ok = 0;
            uint32_t t0 = millis();
            for (int i = 0; i < n; i++) { if (read30((uint8_t)id)) ok++; delay(5); }
            uint32_t el = millis() - t0;
            Serial.printf("{\"read\":\"M%d\",\"n\":%d,\"ok\":%d,\"ms\":%u,\"txFailDelta\":%u}\n",
                          id, n, ok, el, can.getTxFailCount() - fail0);
            printStat("post-R");
        } else Serial.println("{\"error\":\"usage R id n\"}");
    } else if (s.startsWith("A ")) {
        int id = 0, xx = 0, yy = 0;
        if (sscanf(s.c_str(), "A %d %d %d", &id, &xx, &yy) == 3) {
            uint8_t cmd[4] = { 0x8C, (uint8_t)xx, (uint8_t)yy,
                               (uint8_t)((id + 0x8C + xx + yy) & 0xFF) };
            uint8_t r = can.sendMsgBuf((uint8_t)id, 0, 4, cmd);
            Serial.printf("{\"tx8C\":\"M%d\",\"xx\":%d,\"yy\":%d,\"sent\":%d}\n", id, xx, yy, r == TWAI_CAN_OK ? 1 : 0);
            int got = drainPrint(150, "ack8C");
            Serial.printf("{\"ack8C_frames\":%d}\n", got);
        } else Serial.println("{\"error\":\"usage A id xx yy\"}");
    } else if (s.startsWith("T ")) {
        int id = 0, code = 0;
        if (sscanf(s.c_str(), "T %d %d", &id, &code) == 2 && code >= 0 && code <= 3) {
            uint8_t cmd[3] = { 0x8A, (uint8_t)code, (uint8_t)((id + 0x8A + code) & 0xFF) };
            uint8_t r = can.sendMsgBuf((uint8_t)id, 0, 3, cmd);
            Serial.printf("{\"tx8A\":\"M%d\",\"code\":%d,\"sent\":%d}\n", id, code, r == TWAI_CAN_OK ? 1 : 0);
            int got = drainPrint(150, "ack8A");
            Serial.printf("{\"ack8A_frames\":%d}\n", got);
        } else Serial.println("{\"error\":\"usage T id code(2=500K,3=1M)\"}");
    } else if (s.startsWith("R2 ")) {
        // 診斷讀：R2 id n gapMs timeoutMs — 每筆結果字元（.=ok x=timeout c=壞CRC o=異幀）+ 延遲統計
        int id = 0, n = 0, gap = 5, tmo = 25;
        if (sscanf(s.c_str(), "R2 %d %d %d %d", &id, &n, &gap, &tmo) >= 2 && n > 0 && n <= 500) {
            printStat("pre-R2");
            char pat[501];
            uint32_t latSum = 0, latMax = 0; int okc = 0, oddPrinted = 0;
            for (int i = 0; i < n; i++) {
                uint8_t cmd[2] = { 0x30, (uint8_t)((id + 0x30) & 0xFF) };
                uint32_t t0 = millis();
                char r = 'x';
                if (can.sendMsgBuf((uint8_t)id, 0, 2, cmd) != TWAI_CAN_OK) r = 'T';  // TX fail
                else while (millis() - t0 < (uint32_t)tmo) {
                    if (can.checkReceive() != TWAI_CAN_MSGAVAIL) continue;
                    unsigned long rid; uint8_t len, rx[8];
                    if (can.readMsgBuf(&rid, &len, rx) != TWAI_CAN_OK) continue;
                    if ((rid & 0x7FF) != (unsigned)id || len < 8 || rx[0] != 0x30) {
                        r = 'o';
                        if (oddPrinted < 8) {   // 異幀原樣印（限量）
                            Serial.printf("{\"odd\":\"0x%02lX len%u:", rid & 0x7FF, len);
                            for (int k = 0; k < len; k++) Serial.printf(" %02X", rx[k]);
                            Serial.println("\"}");
                            oddPrinted++;
                        }
                        continue;
                    }
                    uint8_t crc = id; for (int k = 0; k < 7; k++) crc += rx[k];
                    r = ((crc & 0xFF) == rx[7]) ? '.' : 'c';
                    break;
                }
                uint32_t lat = millis() - t0;
                if (r == '.') { okc++; latSum += lat; if (lat > latMax) latMax = lat; }
                pat[i] = r;
                delay(gap);
            }
            pat[n] = 0;
            Serial.printf("{\"r2\":\"M%d\",\"n\":%d,\"ok\":%d,\"gap\":%d,\"tmo\":%d,"
                          "\"latAvg\":%.1f,\"latMax\":%u,\"pat\":\"%s\"}\n",
                          id, n, okc, gap, tmo, okc ? (float)latSum / okc : -1.0f, latMax, pat);
            printStat("post-R2");
        } else Serial.println("{\"error\":\"usage R2 id n gapMs timeoutMs\"}");
    } else if (s.startsWith("W ")) {
        // 暖機：連發不等回覆，逼混速旁觀節點 REC 爬滿進 error-passive（之後閉嘴）。
        // TwaiCanCompat 的 bus-off recovery / TX 死鎖斬斷在 sendMsgBuf 入口自動運作。
        int id = 0, n = 0;
        if (sscanf(s.c_str(), "W %d %d", &id, &n) == 2 && n > 0 && n <= 5000) {
            printStat("pre-W");
            uint8_t cmd[2] = { 0x30, (uint8_t)((id + 0x30) & 0xFF) };
            for (int i = 0; i < n; i++) {
                can.sendMsgBuf((uint8_t)id, 0, 2, cmd);
                delay(5);
                if ((i + 1) % 50 == 0) {
                    Serial.printf("{\"warm\":%d,\"tec\":%u,\"rec\":%u,\"txFail\":%u}\n",
                                  i + 1, can.getTEC(), can.getREC(), can.getTxFailCount());
                }
                // 排空避免 RX queue 滿
                unsigned long rid; uint8_t len, rx[8];
                while (can.checkReceive() == TWAI_CAN_MSGAVAIL) can.readMsgBuf(&rid, &len, rx);
            }
            printStat("post-W");
        } else Serial.println("{\"error\":\"usage W id n\"}");
    } else if (s.length()) {
        Serial.println("{\"error\":\"cmds: B500 B1M S Q R A T W\"}");
    }
}

void setup() {
    Serial.begin(460800);
    delay(400);
    Serial.println("\n{\"probe\":\"bitrate_probe\",\"pins\":\"TX21/RX22\",\"boot_bps\":500000}");
    // 裸 TWAI 診斷：印 install/start 的 esp_err 碼（定位 CAN init failed 根因）
    {
        twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_PIN, TWAI_RX_PIN, TWAI_MODE_NORMAL);
        twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
        twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
        esp_err_t ei = twai_driver_install(&g, &t, &f);
        esp_err_t es = (ei == ESP_OK) ? twai_start() : ESP_FAIL;
        Serial.printf("{\"diag\":\"raw twai\",\"install\":\"0x%X(%s)\",\"start\":\"0x%X(%s)\",\"heap\":%u}\n",
                      ei, esp_err_to_name(ei), es, esp_err_to_name(es), ESP.getFreeHeap());
        if (ei == ESP_OK) { twai_stop(); twai_driver_uninstall(); }
    }
    if (!can.begin(500000)) { Serial.println("{\"fatal\":\"twai begin failed\"}"); return; }
    printStat("boot");
    Serial.println("{\"ready\":1}");
}

void loop() {
    static String buf;
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') { if (buf.length()) { handleLine(buf); buf = ""; } }
        else buf += c;
    }
    delay(2);
}
