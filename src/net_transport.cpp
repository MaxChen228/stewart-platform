#include "net_transport.h"
#include <WiFi.h>
#include <Preferences.h>

NetCfg netCfg;

void NetCfg::load() {
    Preferences prefs;
    prefs.begin("netcfg", true);
    ssid    = prefs.getString("ssid", "");
    pass    = prefs.getString("pass", "");
    enabled = prefs.getBool("en", false);
    prefs.end();
}

void NetCfg::save() const {
    Preferences prefs;
    prefs.begin("netcfg", false);
    prefs.putString("ssid", ssid);
    prefs.putString("pass", pass);
    prefs.putBool("en", enabled);
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
    Serial.printf("{\"status\":\"wifi boot\",\"connected\":%d,\"ip\":\"%s\"}\n",
                  ok ? 1 : 0, WiFi.localIP().toString().c_str());
}

// WIFI? 狀態回報（不回 pass 明文）
static void netReportStatus() {
    Serial.printf("{\"wifi\":{\"enabled\":%d,\"connected\":%d,\"ssid\":\"%s\",\"ip\":\"%s\",\"rssi\":%d}}\n",
                  netCfg.enabled ? 1 : 0,
                  WiFi.status() == WL_CONNECTED ? 1 : 0,
                  netCfg.ssid.c_str(),
                  WiFi.localIP().toString().c_str(),
                  (int)WiFi.RSSI());
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
        Serial.printf("{\"status\":\"wifi on\",\"connected\":%d,\"ip\":\"%s\"}\n",
                      ok ? 1 : 0, WiFi.localIP().toString().c_str());
        return true;
    }
    if (cmd == "WIFIOFF") {
        netCfg.enabled = false;
        netCfg.save();
        netWifiStop();
        Serial.println("{\"status\":\"wifi off\"}");
        return true;
    }
    if (cmd.startsWith("WIFI ")) {
        // WIFI <ssid> <pass>：naive space-split（含空格者不支援）
        String rest = cmd.substring(5);
        rest.trim();
        int sp = rest.indexOf(' ');
        if (sp < 0) {
            Serial.println("{\"error\":\"usage: WIFI <ssid> <pass>\"}");
            return true;
        }
        netCfg.ssid = rest.substring(0, sp);
        netCfg.pass = rest.substring(sp + 1);
        netCfg.save();
        Serial.printf("{\"status\":\"wifi creds saved\",\"ssid\":\"%s\"}\n", netCfg.ssid.c_str());
        return true;
    }
    return false;
}
