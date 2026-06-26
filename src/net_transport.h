#pragma once
#include <Arduino.h>

// ===== WiFi TCP 傳輸層（與 USB Serial 並存）=====
// Tier A：ESP32 開 raw TCP server，協議與 USB 逐字相同（指令 ASCII 行、遙測 JSON 行）。
// P1 範圍：NVS 憑證 + WiFi bring-up + WIFI 系列指令。netTask/DualPrint/queue 於後續 phase 加入。
//
// NVS namespace 用 "netcfg"，與 encoder 的 "stewart" 分離，避免撞 key。
// 憑證以個別 key 存（putString/putBool），schema 演進時加欄位不會讓既有讀取失敗。

struct NetCfg {
    String ssid;
    String pass;
    bool   enabled = false;   // WIFION/WIFIOFF；開機時據此決定是否自動連線

    void load();              // 從 NVS "netcfg" 讀；缺值用預設
    void save() const;        // 寫回 NVS "netcfg"
};

extern NetCfg netCfg;

// WiFi STA bring-up：mode(STA) + setSleep(false)（關 modem sleep 避免 +100ms 延遲）
// + WiFi.begin。非阻塞啟動後做有界等待，回傳是否在 waitMs 內連上。
// ssid 為空則直接回 false（不啟動）。
bool netWifiBegin(uint32_t waitMs = 8000);
void netWifiStop();           // 斷線並關閉 STA

// setup() 末呼叫：載入 NVS 憑證，若 enabled 則自動連線並印 boot 狀態。
void netBoot();

// 攔截 WIFI 系列指令：WIFI <ssid> <pass> / WIFION / WIFIOFF / WIFI?
// 有處理回 true（呼叫端據此短路 dispatch）；非 WIFI 指令回 false。
// 限制：naive space-split，含空格的 SSID/密碼不支援。
bool netHandleCommand(const String& cmd);
