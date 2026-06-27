#include "net_transport.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

NetCfg netCfg;

// ===== TCP 傳輸基礎設施（P2）=====
static const uint16_t TCP_PORT = 3333;

// 固定大小 by-value 項：避免 heap 碎片，drop 時無 double-free 風險。
struct OutLine { char s[1024]; };
struct NetCmd  { char s[128]; };       // 指令行最長 ~ "W f f f f f f ms" < 128

static QueueHandle_t qOut = nullptr;   // loop(core1) → netTask(core0)，遙測，len=32 (~1s@33Hz)
static QueueHandle_t qIn  = nullptr;   // netTask(core0) → loop(core1)，指令，len=16

// 連線狀態（core0 寫、core1 讀；單字組原子）
volatile bool     netConnected = false;
volatile uint32_t lastNetRxMs  = 0;
static volatile uint32_t netClientSeq = 0;
static volatile uint32_t netOutLines = 0;
static volatile uint32_t netOutDrop = 0;
static volatile uint32_t netOutSkip = 0;
static volatile uint32_t netOutBackpressure = 0;
static volatile uint32_t netOutShortWrite = 0;
static volatile uint32_t netLastWriteAvail = 0;
static volatile uint32_t netClientReject = 0;
static volatile uint32_t netClientPreempt = 0;
static volatile uint32_t netClientStop = 0;
static volatile uint32_t netWifiDrop = 0;
static volatile uint32_t netInLines = 0;
static volatile uint32_t netInDrop = 0;
static volatile uint32_t netInTooLong = 0;
static const char* netCloseReason = "none";
static SemaphoreHandle_t outMux = nullptr;

static void lockOut() {
    if (!outMux) outMux = xSemaphoreCreateRecursiveMutex();
    if (outMux) xSemaphoreTakeRecursive(outMux, portMAX_DELAY);
}

static void unlockOut() {
    if (outMux) xSemaphoreGiveRecursive(outMux);
}

#ifndef OTA_HOSTNAME
#define OTA_HOSTNAME "stewart"
#endif

#ifndef OTA_PASSWORD
#define OTA_PASSWORD ""
#endif

static bool otaStarted = false;
static volatile bool otaInProgress = false;
static void (*otaStartCallback)() = nullptr;

// WiFi/TCP is an observation path, not the control loop. Keep it intentionally
// lossy so slow browsers, TCP backpressure, or reconnect storms cannot make the
// ESP32 tear down an otherwise healthy control session.
static constexpr uint32_t NET_TELEM_MIN_MS = 50;  // cap full telemetry at ~20 Hz over TCP
static constexpr uint32_t NET_CLIENT_STALE_MS = 3000;

static bool isTelemetryLine(const char* s) {
    return s && s[0] == '{' && s[1] == '"' && s[2] == 'a' && s[3] == '"' && s[4] == ':';
}

static void stopNetClient(WiFiClient& client, const char* reason) {
    if (client) client.stop();
    netConnected = false;
    netClientStop++;
    netCloseReason = reason ? reason : "stop";
}

void netSetOtaStartCallback(void (*cb)()) {
    otaStartCallback = cb;
}

void netOtaBegin() {
    if (otaStarted || WiFi.status() != WL_CONNECTED) return;
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    if (String(OTA_PASSWORD).length() > 0) ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA
        .onStart([]() {
            otaInProgress = true;
            if (otaStartCallback) otaStartCallback();
            Out.println("{\"ota\":\"start\"}");
        })
        .onEnd([]() {
            Out.println("{\"ota\":\"end\"}");
            otaInProgress = false;
        })
        .onError([](ota_error_t error) {
            Out.printf("{\"ota\":\"error\",\"code\":%u}\n", (unsigned)error);
            otaInProgress = false;
        });
    ArduinoOTA.begin();
    otaStarted = true;
    Out.printf("{\"ota\":\"ready\",\"host\":\"%s\",\"ip\":\"%s\",\"auth\":%d}\n",
               OTA_HOSTNAME, WiFi.localIP().toString().c_str(), String(OTA_PASSWORD).length() > 0 ? 1 : 0);
}

void netOtaHandle() {
    if (WiFi.status() != WL_CONNECTED) return;
    if (!otaStarted) netOtaBegin();
    ArduinoOTA.handle();
}

bool netOtaBusy() {
    return otaInProgress;
}

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
        netOutDrop++;
    }
}

size_t DualPrint::write(uint8_t c) {
    lockOut();
    Serial.write(c);                   // (a) 永遠寫 USB（此處保留 Serial，是真實 sink；byte-parity）
    if (c == '\n') {
        flushLine();
        unlockOut();
        return 1;
    }   // '\n' 不入緩衝，由 flushLine 統一補
    if (n < sizeof(line) - 1) line[n++] = c;    // (b) 只累積內容
    unlockOut();
    return 1;
}

size_t DualPrint::write(const uint8_t* buf, size_t len) {
    if (!buf || len == 0) return 0;
    lockOut();
    Serial.write(buf, len);             // bulk write: avoid one UART mutex/blocking call per byte
    for (size_t i = 0; i < len; i++) {
        uint8_t c = buf[i];
        if (c == '\n') { flushLine(); continue; }
        if (n < sizeof(line) - 1) line[n++] = c;
    }
    unlockOut();
    return len;
}

// ----- netTask（core0）：WiFi 連上後開 TCP server，排空遙測 queue 寫 socket -----
// P2 僅出站（遙測）；入站指令於 P3 加入。馬達/SPI 全留 core1，netTask 絕不碰。
static void netTask(void* arg) {
    WiFiServer server(TCP_PORT);
    WiFiClient client;
    bool started = false;
    uint32_t lastNetTxMs = 0;
    char  inbuf[128];           // 入站行緩衝（單 client、netTask 獨用）
    size_t inn = 0;
    bool inOverflow = false;
    for (;;) {
        if (WiFi.status() != WL_CONNECTED) {
            if (started) {
                stopNetClient(client, "wifi_down");
                server.end();
                started = false;
                netWifiDrop++;
            }  // 釋放半開資源
            netConnected = false;
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        if (!started) { server.begin(); server.setNoDelay(true); started = true; }

        uint32_t nowMs = millis();
        bool clientAlive = (client && client.connected());
        // Treat host input, not ESP32 output, as proof of a healthy TCP client.
        // A half-open socket can keep accepting writes into local buffers, which
        // previously protected a ghost client and caused every new dashboard
        // connection to be reset. The Node transport sends a 1Hz newline
        // heartbeat, so a real client stays fresh without issuing commands.
        if (clientAlive && lastNetRxMs && nowMs - lastNetRxMs > NET_CLIENT_STALE_MS) {
            stopNetClient(client, "rx_stale");
            clientAlive = false;
        }

        // single-client, but not naive newest-wins:
        // - a healthy dashboard connection is protected from accidental probes
        // - stale/half-dead clients are replaced so the server can recover
        if (server.hasClient()) {
            WiFiClient c = server.available();
            if (c) {
                if (clientAlive) {
                    c.stop();
                    netClientReject++;
                } else {
                    if (client) {
                        stopNetClient(client, "preempt");
                        netClientPreempt++;
                    }
                    client = c;
                    client.setNoDelay(true);
                    inn = 0;
                    inOverflow = false;
                    lastNetRxMs = nowMs;
                    lastNetTxMs = nowMs;
                    netCloseReason = "none";
                    netClientSeq++;
                    clientAlive = true;
                }
            }
        }
        netConnected = (client && client.connected());

        // 出站優先：先排空遙測，避免入站背壓餓死遙測（P3 review nit）。
        // 無 client 也排空（丟棄）→ queue 不堆積。
        static uint32_t lastTeleWriteMs = 0;
        OutLine ol;
        while (xQueueReceive(qOut, &ol, 0) == pdTRUE) {
            if (netConnected) {
                size_t len = strlen(ol.s);
                bool tele = isTelemetryLine(ol.s);
                uint32_t now = millis();
                if (tele && now - lastTeleWriteMs < NET_TELEM_MIN_MS) {
                    netOutSkip++;
                    continue;
                }
                int avail = client.availableForWrite();
                netLastWriteAvail = avail > 0 ? (uint32_t)avail : 0;
                if (avail > 0 && (size_t)avail < len) {
                    netOutBackpressure++;
                    netOutDrop++;
                    continue;
                }
                size_t n = client.write((const uint8_t*)ol.s, len);   // ol.s 已含 '\n'
                if (n == len) {
                    netOutLines++;
                    lastNetTxMs = now;
                    if (tele) lastTeleWriteMs = now;
                } else if (n == 0) {
                    netOutBackpressure++;
                    netOutDrop++;
                } else {
                    // A partial JSON line corrupts framing; close only this rare
                    // case and let the host reconnect. Backpressure before write
                    // is handled by dropping whole lines above.
                    netOutShortWrite++;
                    netOutDrop++;
                    stopNetClient(client, "short_write");
                    break;
                }
            }
        }

        // 入站：socket → 行切割 → qIn。send timeout=0（qIn 滿則丟，不阻塞 netTask）——
        // 唯有 loop 卡住、16 條未處理才會丟，遠優於遙測連帶停擺。
        while (netConnected && client.available()) {
            int ch = client.read();
            if (ch < 0) break;
            lastNetRxMs = millis();      // 任何 byte 都更新心跳基準
            if (ch == '\n' || ch == '\r') {
                if (inOverflow) {
                    inn = 0;
                    inOverflow = false;
                } else if (inn > 0) {
                    NetCmd nc;
                    size_t m = (inn < sizeof(nc.s)) ? inn : sizeof(nc.s) - 1;
                    memcpy(nc.s, inbuf, m); nc.s[m] = '\0'; inn = 0;
                    if (qIn && xQueueSend(qIn, &nc, 0) == pdTRUE) netInLines++;
                    else netInDrop++;
                }
            } else if (inn < sizeof(inbuf) - 1) {
                inbuf[inn++] = (char)ch;
            } else {
                inOverflow = true;
                netInTooLong++;
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

// mDNS 廣播 stewart.local → ESP32 IP（server 端零設定發現 WiFi 路徑）。
// 連線/重連成功後單點呼叫；MDNS.end() 先清，重連時可冪等重設。
static void netAdvertiseMDNS() {
    if (WiFi.status() != WL_CONNECTED) return;
    MDNS.end();
    if (MDNS.begin("stewart"))                  // → stewart.local 解析到本機 IP
        MDNS.addService("stewart", "tcp", TCP_PORT);   // _stewart._tcp.local:3333 供發現
}

bool netWifiBegin(uint32_t waitMs) {
    if (netCfg.ssid.length() == 0) return false;
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);                       // 關 modem sleep（即時控制必要）
    WiFi.begin(netCfg.ssid.c_str(), netCfg.pass.c_str());
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < waitMs) delay(100);
    bool ok = (WiFi.status() == WL_CONNECTED);
    if (ok) {
        netAdvertiseMDNS();                     // boot 與 WIFION 都經此 → mDNS 單一啟動點
        netOtaBegin();
    }
    return ok;
}

void netWifiStop() {
    otaStarted = false;
    otaInProgress = false;
    ArduinoOTA.end();
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
    String ssidEsc;
    ssidEsc.reserve(netCfg.ssid.length() + 8);
    for (size_t i = 0; i < netCfg.ssid.length(); i++) {
        char c = netCfg.ssid[i];
        if (c == '"' || c == '\\') { ssidEsc += '\\'; ssidEsc += c; }
        else if ((uint8_t)c < 0x20) ssidEsc += '?';
        else ssidEsc += c;
    }
    uint32_t now = millis();
    uint32_t age = lastNetRxMs ? now - lastNetRxMs : 0;
    unsigned qOutDepth = qOut ? uxQueueMessagesWaiting(qOut) : 0;
    unsigned qInDepth = qIn ? uxQueueMessagesWaiting(qIn) : 0;
    Out.printf("{\"wifi\":{\"enabled\":%d,\"connected\":%d,\"ssid\":\"%s\",\"ip\":\"%s\",\"rssi\":%d,"
               "\"heap\":%u,\"min_heap\":%u,\"fs\":%u,\"hb_ms\":%u,"
               "\"ota\":{\"ready\":%d,\"busy\":%d,\"host\":\"%s\"},"
               "\"tcp\":{\"client\":%d,\"seq\":%u,\"last_rx_age_ms\":%u,"
               "\"qout\":%u,\"qin\":%u,\"out\":%u,\"out_drop\":%u,\"out_skip\":%u,"
               "\"out_bp\":%u,\"out_short\":%u,\"wr_avail\":%u,"
               "\"reject\":%u,\"preempt\":%u,\"stop\":%u,\"wifi_drop\":%u,\"close\":\"%s\","
               "\"in\":%u,\"in_drop\":%u,\"in_long\":%u}}}\n",
                  netCfg.enabled ? 1 : 0,
                  WiFi.status() == WL_CONNECTED ? 1 : 0,
                  ssidEsc.c_str(),
                  WiFi.localIP().toString().c_str(),
                  (int)WiFi.RSSI(),
                  (unsigned)ESP.getFreeHeap(),
                  (unsigned)ESP.getMinFreeHeap(),
                  (unsigned)netCfg.failsafe,
                  (unsigned)netCfg.hbTimeoutMs,
                  otaStarted ? 1 : 0,
                  otaInProgress ? 1 : 0,
                  OTA_HOSTNAME,
                  netConnected ? 1 : 0,
                  (unsigned)netClientSeq,
                  (unsigned)age,
                  qOutDepth,
                  qInDepth,
                  (unsigned)netOutLines,
                  (unsigned)netOutDrop,
                  (unsigned)netOutSkip,
                  (unsigned)netOutBackpressure,
                  (unsigned)netOutShortWrite,
                  (unsigned)netLastWriteAvail,
                  (unsigned)netClientReject,
                  (unsigned)netClientPreempt,
                  (unsigned)netClientStop,
                  (unsigned)netWifiDrop,
                  netCloseReason,
                  (unsigned)netInLines,
                  (unsigned)netInDrop,
                  (unsigned)netInTooLong);
}

bool netHandleCommand(const String& cmd, bool fromNet, bool motionActive) {
    (void)fromNet;
    if (cmd == "WIFI?") {
        netReportStatus();
        return true;
    }
    if (cmd == "WIFION") {
        if (motionActive) {
            Out.println("{\"error\":\"WIFION rejected while position control is active\"}");
            return true;
        }
        netCfg.enabled = true;
        netCfg.save();
        bool ok = netWifiBegin();
        Out.printf("{\"status\":\"wifi on\",\"connected\":%d,\"ip\":\"%s\"}\n",
                      ok ? 1 : 0, WiFi.localIP().toString().c_str());
        return true;
    }
    if (cmd == "WIFIOFF") {
        if (motionActive) {
            Out.println("{\"error\":\"WIFIOFF rejected while position control is active\"}");
            return true;
        }
        netCfg.enabled = false;
        netCfg.save();
        netWifiStop();
        Out.println("{\"status\":\"wifi off\"}");
        return true;
    }
    if (cmd.startsWith("WIFI ")) {
        if (motionActive) {
            Out.println("{\"error\":\"WIFI creds rejected while position control is active\"}");
            return true;
        }
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
