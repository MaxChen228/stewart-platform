#pragma once
#include <Arduino.h>

// ===== WiFi TCP 傳輸層（與 USB Serial 並存）=====
// Tier A：ESP32 開 raw TCP server，協議與 USB 逐字相同（指令 ASCII 行、遙測 JSON 行）。
// P1 範圍：NVS 憑證 + WiFi bring-up + WIFI 系列指令。netTask/DualPrint/queue 於後續 phase 加入。
//
// NVS namespace 用 "netcfg"，與 encoder 的 "stewart" 分離，避免撞 key。
// 憑證以個別 key 存（putString/putBool），schema 演進時加欄位不會讓既有讀取失敗。

// 失效保護策略（P4）。預設 HOLD-current：snapshot 當前實際角度保持，
// 靠馬達內環保形、不靠會發散的 ESP32 外環，且非硬撐追逐中的 target。
enum Failsafe : uint8_t { FS_HOLD_CURRENT = 0, FS_DISABLE = 1 };

struct NetCfg {
    String   ssid;
    String   pass;
    bool     enabled     = false;   // WIFION/WIFIOFF；開機時據此決定是否自動連線
    uint8_t  failsafe    = FS_HOLD_CURRENT;  // FS 指令；斷線安全態
    uint32_t hbTimeoutMs = 0;       // HB 指令；心跳逾時(ms)，0=停用心跳檢查（僅靠 socket-close）
    // TCP 最小 shared-secret（NETKEY 指令，USB-only 設定）。空 = 開放放行（向後相容，升級不鎖死）。
    // 定長 char[]（非 String）：netTask(core0) 讀、dispatch(core1) 寫，定長緩衝跨核讀寫不會因
    // String 重配/指標交換 crash；最壞只是 NETKEY 改寫的微秒窗內比對撕裂，reconnect 自癒。
    char     netkey[33]  = "";       // 最長 32 字元 token + '\0'

    void load();              // 從 NVS "netcfg" 讀；缺值用預設
    void save() const;        // 寫回 NVS "netcfg"
};

extern NetCfg netCfg;

// ===== 連線狀態（P4，core0 netTask 維護，core1 checkFailsafe 讀）=====
// 單字組對齊 → ESP32 上讀寫原子，免 mutex。
extern volatile bool     netConnected;   // WiFi 已連 且 TCP client live
extern volatile uint32_t lastNetRxMs;    // 最後一次從 socket 收到 byte 的時刻（心跳基準）
extern volatile uint32_t netTaskHeartbeatMs;  // netTask 每輪更新（millis）；core1 watchdog 偵測 netTask wedge

// ===== DualPrint：輸出 fan-out（P2）=====
// 繼承 Arduino Print，所有 print/printf/println/write 都 funnel 經 write()：
//   (a) 永遠寫 USB Serial（debug/upload/failover 主路徑不丟）
//   (b) 累積行緩衝，遇 '\n' 把整行推進 outbound 遙測 queue（滿則丟最舊）
// 並發不變量：Out 可能被 setup/control/telemetry callback 寫；DualPrint 內部用 mutex
// 保護行緩衝。netTask（core0）只「讀」queue，絕不呼叫 Out.*。
// 限制：單行 >NET_OUT_LINE_BYTES 會被截斷；telemetry 應用 writeLine() 原子送出。
static constexpr size_t NET_OUT_LINE_BYTES = 1536;

class DualPrint : public Print {
    char   line[NET_OUT_LINE_BYTES];
    size_t n = 0;
    void flushLine();
public:
    size_t write(uint8_t c) override;
    size_t write(const uint8_t* buf, size_t len) override;
    size_t writeLine(const char* s);
};

extern DualPrint Out;

// 建立 in/out queue 並啟動 core0 的 netTask（TCP server）。setup() 末呼叫。
void netInit();

// Optional OTA upload service. Safe to call repeatedly after WiFi is connected.
// The callback runs in the low-priority loop/I/O context from netOtaHandle();
// it should request motion-stop work instead of touching motors directly.
void netOtaBegin();
void netOtaHandle();
bool netOtaBusy();
void netSetOtaStartCallback(void (*cb)());

// 非阻塞取一條 TCP 來源指令（P3）：有則填入 out 回 true，否則 false。
// loop 端呼叫，語意同 Serial.available 輪詢；指令由 netTask 從 socket 收進 qIn。
bool netNextCommand(String& out);

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
bool netHandleCommand(const String& cmd, bool fromNet, bool motionActive);
