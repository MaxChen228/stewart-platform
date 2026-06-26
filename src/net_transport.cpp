#include "net_transport.h"
#include <WiFi.h>
#include <Preferences.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

NetCfg netCfg;

// ===== TCP 傳輸基礎設施（P2）=====
static const uint16_t TCP_PORT = 3333;

// 固定大小 by-value 項：避免 heap 碎片，drop 時無 double-free 風險。
struct OutLine { char s[640]; };
struct NetCmd  { char s[128]; };       // 指令行最長 ~ "W f f f f f f ms" < 128

static QueueHandle_t qOut = nullptr;   // loop(core1) → netTask(core0)，遙測，len=32 (~1s@33Hz)
static QueueHandle_t qIn  = nullptr;   // netTask(core0) → loop(core1)，指令，len=16

// 連線狀態（core0 寫、core1 讀；單字組原子）
volatile bool     netConnected = false;
volatile uint32_t lastNetRxMs  = 0;

// ----- DualPrint：唯一 writer = core1（loop/setup），免 mutex -----
DualPrint Out;

void DualPrint::flushLine() {
    if (!qOut) { n = 0; return; }      // netInit 前（setup 早期）只走 Serial，不入隊
    OutLine ol;
    size_t m = (n < sizeof(ol.s) - 1) ? n : sizeof(ol.s) - 2;  // 留 1 格給 '\n'
    memcpy(ol.s, line, m);
    ol.s[m]     = '\n';                // 一律補行尾：截斷的超長行（X-diag）才不會 TCP 黏幀
    ol.s[m + 1] = '\0';
    n = 0;
    if (xQueueSend(qOut, &ol, 0) == errQUEUE_FULL) {   // drop-oldest（僅 producer/core1 做此 dance）
        OutLine drop;
        xQueueReceive(qOut, &drop, 0);
        xQueueSend(qOut, &ol, 0);
    }
}

size_t DualPrint::write(uint8_t c) {
    Serial.write(c);                   // (a) 永遠寫 USB（此處保留 Serial，是真實 sink；byte-parity）
    if (c == '\n') { flushLine(); return 1; }   // '\n' 不入緩衝，由 flushLine 統一補
    if (n < sizeof(line) - 1) line[n++] = c;    // (b) 只累積內容
    return 1;
}

size_t DualPrint::write(const uint8_t* buf, size_t len) {
    for (size_t i = 0; i < len; i++) write(buf[i]);   // 逐 byte；33Hz×400B 無效能顧慮
    return len;
}

// ----- netTask（core0）：WiFi 連上後開 TCP server，排空遙測 queue 寫 socket -----
// P2 僅出站（遙測）；入站指令於 P3 加入。馬達/SPI 全留 core1，netTask 絕不碰。
static void netTask(void* arg) {
    WiFiServer server(TCP_PORT);
    WiFiClient client;
    bool started = false;
    char  inbuf[128];           // 入站行緩衝（單 client、netTask 獨用）
    size_t inn = 0;
    for (;;) {
        if (WiFi.status() != WL_CONNECTED) {
            if (started) { client.stop(); server.end(); started = false; }  // 釋放半開資源
            netConnected = false;
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        if (!started) { server.begin(); server.setNoDelay(true); started = true; }

        if (!client || !client.connected()) {
            WiFiClient c = server.available();
            if (c) { client = c; client.setNoDelay(true); inn = 0; lastNetRxMs = millis(); }
        }
        netConnected = (client && client.connected());

        // 出站優先：先排空遙測，避免入站背壓餓死遙測（P3 review nit）。
        // 無 client 也排空（丟棄）→ queue 不堆積。
        OutLine ol;
        while (xQueueReceive(qOut, &ol, 0) == pdTRUE) {
            if (netConnected) client.print(ol.s);   // ol.s 已含 '\n'
        }

        // 入站：socket → 行切割 → qIn。send timeout=0（qIn 滿則丟，不阻塞 netTask）——
        // 唯有 loop 卡住、16 條未處理才會丟，遠優於遙測連帶停擺。
        while (netConnected && client.available()) {
            int ch = client.read();
            if (ch < 0) break;
            lastNetRxMs = millis();      // 任何 byte 都更新心跳基準
            if (ch == '\n' || ch == '\r') {
                if (inn > 0) {
                    NetCmd nc;
                    size_t m = (inn < sizeof(nc.s)) ? inn : sizeof(nc.s) - 1;
                    memcpy(nc.s, inbuf, m); nc.s[m] = '\0'; inn = 0;
                    if (qIn) xQueueSend(qIn, &nc, 0);
                }
            } else if (inn < sizeof(inbuf) - 1) {
                inbuf[inn++] = (char)ch;
            }   // 行超長則丟棄多餘 byte（指令 <128，正常不會觸發）
        }
        vTaskDelay(pdMS_TO_TICKS(2));   // 讓出 CPU
    }
}

// loop(core1) 非阻塞取一條 TCP 指令
bool netNextCommand(String& out) {
    if (!qIn) return false;
    NetCmd nc;
    if (xQueueReceive(qIn, &nc, 0) != pdTRUE) return false;
    out = nc.s;
    out.trim();              // 與 USB 路徑（handleSerial 的 cmd.trim）對稱，免尾空白失配
    return true;
}

void netInit() {
    qOut = xQueueCreate(32, sizeof(OutLine));
    qIn  = xQueueCreate(16, sizeof(NetCmd));
    xTaskCreatePinnedToCore(netTask, "netTask", 8192, nullptr, 1, nullptr, 0);  // core0、低優先不搶控制環
}

void NetCfg::load() {
    Preferences prefs;
    prefs.begin("netcfg", true);
    ssid        = prefs.getString("ssid", "");
    pass        = prefs.getString("pass", "");
    enabled     = prefs.getBool("en", false);
    failsafe    = prefs.getUChar("fs", FS_HOLD_CURRENT);
    hbTimeoutMs = prefs.getUInt("hb", 0);
    prefs.end();
}

void NetCfg::save() const {
    Preferences prefs;
    prefs.begin("netcfg", false);
    prefs.putString("ssid", ssid);
    prefs.putString("pass", pass);
    prefs.putBool("en", enabled);
    prefs.putUChar("fs", failsafe);
    prefs.putUInt("hb", hbTimeoutMs);
    prefs.end();
}

bool netWifiBegin(uint32_t waitMs) {
    if (netCfg.ssid.length() == 0) return false;
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);                       // 關 modem sleep（即時控制必要）
    WiFi.begin(netCfg.ssid.c_str(), netCfg.pass.c_str());
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < waitMs) delay(100);
    return WiFi.status() == WL_CONNECTED;
}

void netWifiStop() {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
}

void netBoot() {
    netCfg.load();
    if (!netCfg.enabled) return;
    bool ok = netWifiBegin();
    Out.printf("{\"status\":\"wifi boot\",\"connected\":%d,\"ip\":\"%s\"}\n",
                  ok ? 1 : 0, WiFi.localIP().toString().c_str());
}

// WIFI? 狀態回報（不回 pass 明文）。heap 欄供 P2 記憶體預算驗證——
// 放這裡而非遙測主幀，以保遙測 byte-parity（前端解析依賴主幀格式不變）。
static void netReportStatus() {
    Out.printf("{\"wifi\":{\"enabled\":%d,\"connected\":%d,\"ssid\":\"%s\",\"ip\":\"%s\",\"rssi\":%d,\"heap\":%u}}\n",
                  netCfg.enabled ? 1 : 0,
                  WiFi.status() == WL_CONNECTED ? 1 : 0,
                  netCfg.ssid.c_str(),
                  WiFi.localIP().toString().c_str(),
                  (int)WiFi.RSSI(),
                  (unsigned)ESP.getFreeHeap());
}

bool netHandleCommand(const String& cmd) {
    if (cmd == "WIFI?") {
        netReportStatus();
        return true;
    }
    if (cmd == "WIFION") {
        netCfg.enabled = true;
        netCfg.save();
        bool ok = netWifiBegin();
        Out.printf("{\"status\":\"wifi on\",\"connected\":%d,\"ip\":\"%s\"}\n",
                      ok ? 1 : 0, WiFi.localIP().toString().c_str());
        return true;
    }
    if (cmd == "WIFIOFF") {
        netCfg.enabled = false;
        netCfg.save();
        netWifiStop();
        Out.println("{\"status\":\"wifi off\"}");
        return true;
    }
    if (cmd.startsWith("WIFI ")) {
        // WIFI <ssid> <pass>：naive space-split（含空格者不支援）
        String rest = cmd.substring(5);
        rest.trim();
        int sp = rest.indexOf(' ');
        if (sp < 0) {
            Out.println("{\"error\":\"usage: WIFI <ssid> <pass>\"}");
            return true;
        }
        netCfg.ssid = rest.substring(0, sp);
        netCfg.pass = rest.substring(sp + 1);
        netCfg.save();
        Out.printf("{\"status\":\"wifi creds saved\",\"ssid\":\"%s\"}\n", netCfg.ssid.c_str());
        return true;
    }
    if (cmd.startsWith("FS ")) {
        // FS <0|1>：斷線安全態（0=HOLD-current 預設 / 1=斷電）
        int v = cmd.substring(3).toInt();
        netCfg.failsafe = (v == FS_DISABLE) ? FS_DISABLE : FS_HOLD_CURRENT;
        netCfg.save();
        Out.printf("{\"status\":\"failsafe\",\"fs\":%d}\n", netCfg.failsafe);
        return true;
    }
    if (cmd.startsWith("HB ")) {
        // HB <ms>：心跳逾時，0=停用心跳檢查（僅靠 socket-close）
        netCfg.hbTimeoutMs = (uint32_t)cmd.substring(3).toInt();
        netCfg.save();
        Out.printf("{\"status\":\"heartbeat\",\"hb_ms\":%u}\n", (unsigned)netCfg.hbTimeoutMs);
        return true;
    }
    return false;
}
