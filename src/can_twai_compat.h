#pragma once

// ESP32 TWAI compatibility backend for the existing MCP_CAN-style API.
//
// This header is intentionally not wired into the default build yet. It is a
// drop-in target for the future SN65HVD230/TJA1051 hardware path while keeping
// Servo42D's upper layer on sendMsgBuf/checkReceive/readMsgBuf.

#include <Arduino.h>
#include <driver/twai.h>
#include <freertos/semphr.h>

#ifndef TWAI_TX_PIN
#define TWAI_TX_PIN GPIO_NUM_21
#endif

#ifndef TWAI_RX_PIN
#define TWAI_RX_PIN GPIO_NUM_22
#endif

#ifndef TWAI_RX_QUEUE_LEN
#define TWAI_RX_QUEUE_LEN 64
#endif

#ifndef TWAI_TX_QUEUE_LEN
#define TWAI_TX_QUEUE_LEN 32
#endif

#ifndef TWAI_DEFAULT_BITRATE
#define TWAI_DEFAULT_BITRATE 500000
#endif

#define TWAI_CAN_OK        0
#define TWAI_CAN_FAIL      1
#define TWAI_CAN_MSGAVAIL  3
#define TWAI_CAN_NOMSG     1

class TwaiCanCompat;

struct TwaiCanGuard {
    TwaiCanCompat& can;
    explicit TwaiCanGuard(TwaiCanCompat& c);
    ~TwaiCanGuard();
};

class TwaiCanCompat {
    twai_message_t cached = {};
    bool hasCached = false;
    bool installed = false;
    SemaphoreHandle_t canMux = nullptr;
    uint32_t bitrate = TWAI_DEFAULT_BITRATE;
    uint32_t lastHealthMs = 0;
    uint32_t recoveryCount = 0;
    uint32_t txStuckSinceMs = 0;
    uint32_t txPurgeCount = 0;
    uint32_t busQuietUntilMs = 0;   // 熔斷後全域 TX 靜默窗：讓對方（馬達）的重傳在空 bus 完成，絞肉機斷燃料
    uint32_t lastTecTripMs = 0;
    uint32_t tecTripCount = 0;

    void ensureMutex() {
        if (!canMux) canMux = xSemaphoreCreateRecursiveMutex();
    }

    // ── bus-off 自動恢復（2026-07-02 實症修）──
    // MCP2515 硬體 bus-off 後自動重入（CAN spec 128×11 recessive）；ESP-IDF TWAI driver
    // 則停在 BUS_OFF 等 host 主動 recovery——缺這段會一次瞬態永久卡死
    // （實症：server 重啟 DTR reset 瞬態 → TEC 飽和 → BUS_OFF；rx 全停、TEC 凍 128、ef=0x94）。
    // 呼叫點：sendMsgBuf / checkReceive 入口，50ms throttle（400Hz 迴圈下 ~20Hz 檢查）。
    // 若物理層真故障會反覆 recovery→bus-off，serial 每次轉換各印一行 warn/info 可辨識。
    void serviceBusHealth() {
        uint32_t now = millis();
        if (now - lastHealthMs < 50) return;
        lastHealthMs = now;
        twai_status_info_t st = {};
        if (twai_get_status_info(&st) != ESP_OK) return;
        if (st.state == TWAI_STATE_BUS_OFF) {
            twai_initiate_recovery();          // → RECOVERING（driver 清 TX queue，等 128×11 recessive）
            recoveryCount++;
            txStuckSinceMs = 0;
            Serial.printf("{\"warn\":\"twai bus-off, recovery #%u initiated\"}\n", recoveryCount);
        } else if (st.state == TWAI_STATE_STOPPED) {
            twai_start();                      // recovery 完成落 STOPPED → 重啟回 RUNNING
            Serial.printf("{\"info\":\"twai running again after recovery #%u\"}\n", recoveryCount);
        } else if (st.state == TWAI_STATE_RUNNING) {
            // ── error-passive TX 死鎖斬斷（2026-07-02 實症；2026-07-02 晚二修）──
            // 42D 主動上報/讀取回覆幀 ID == 指令幀 ID（馬達地址）→ 同 ID 仲裁不分 → bit error；
            // error passive 後 TEC 凍結（CAN spec 特例）+ TWAI 無限重傳 → 一幀卡死 TX queue，
            // 反覆與對方重傳互撞 → 對方 TEC 被磨向 255 → 馬達 bus-off（只能斷電救）。
            // 初版缺陷（實症 564 次 purge 未能救回）：①500ms 門檻 = 每輪先絞 500ms；②斬幀後
            // 立即恢復 TX → 再撞上對方仍在重傳的回覆 → 再絞。二修：門檻 50ms + 斬後靜默 50ms
            // （sendMsgBuf 直接回 FAIL，上層 F5 每 cycle 重發/讀取保持上次值，皆無害）。
            if (st.msgs_to_tx > 0) {
                if (!txStuckSinceMs) txStuckSinceMs = now;
                else if (now - txStuckSinceMs > 50) {
                    twai_stop();               // 清 TX queue（含卡死幀）
                    twai_start();
                    txPurgeCount++;
                    txStuckSinceMs = 0;
                    busQuietUntilMs = now + 50;   // 靜默窗：讓對方重傳在空 bus 完成，絞肉機斷燃料
                    Serial.printf("{\"warn\":\"twai tx-deadlock purge #%u (dropped stuck frame, quiet 50ms)\"}\n", txPurgeCount);
                }
            } else {
                txStuckSinceMs = 0;
            }
            // ── error-passive 早期熔斷 ──：TEC≥128（error-passive）＝絞肉機已啟動的直接證據，
            // 不等 TX 卡滿門檻，立即靜默讓在途回覆消散（rate-limit 200ms 防 thrash；
            // TEC 靠恢復後的成功幀 −1/幀 自然消退）。
            if (st.tx_error_counter >= 128 && now - lastTecTripMs > 200) {
                lastTecTripMs = now;
                tecTripCount++;
                busQuietUntilMs = now + 50;
                Serial.printf("{\"warn\":\"twai error-passive trip #%u (tec=%u, quiet 50ms)\"}\n",
                              tecTripCount, (unsigned)st.tx_error_counter);
            }
        }
    }

    static twai_timing_config_t timingFor(uint32_t bps) {
        twai_timing_config_t t;
        switch (bps) {
            case 1000000: t = TWAI_TIMING_CONFIG_1MBITS();   break;
            case 500000:  t = TWAI_TIMING_CONFIG_500KBITS(); break;
            case 250000:  t = TWAI_TIMING_CONFIG_250KBITS(); break;
            case 125000:  t = TWAI_TIMING_CONFIG_125KBITS(); break;
            default:      t = TWAI_TIMING_CONFIG_500KBITS(); break;
        }
        // 三重取樣（≤500k 合法）：2026-07-02 實測 VP230 TX 路徑邊際——每幀平均重傳 1–2 次
        // 才成功（TEC 12 幀即衝 ~134、RX 恆完美），多數決取樣压 bit error 率。
        if (bps <= 500000) t.triple_sampling = true;
        return t;
    }

public:
    explicit TwaiCanCompat(uint8_t /*unusedCs*/) {}

    void lock() {
        ensureMutex();
        if (canMux) xSemaphoreTakeRecursive(canMux, portMAX_DELAY);
    }

    void unlock() {
        if (canMux) xSemaphoreGiveRecursive(canMux);
    }

    bool begin(uint32_t bps = TWAI_DEFAULT_BITRATE) {
        TwaiCanGuard _g(*this);
        if (installed) {
            twai_stop();
            twai_driver_uninstall();
            installed = false;
        }

        bitrate = bps;
        twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_PIN, TWAI_RX_PIN, TWAI_MODE_NORMAL);
        g.rx_queue_len = TWAI_RX_QUEUE_LEN;
        g.tx_queue_len = TWAI_TX_QUEUE_LEN;
        twai_timing_config_t t = timingFor(bps);
        twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

        if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
        installed = true;
        hasCached = false;
        if (twai_start() != ESP_OK) {
            twai_driver_uninstall();
            installed = false;
            return false;
        }
        return true;
    }

    uint32_t getBitrate() const { return bitrate; }
    const char* backendName() const { return "twai"; }

    uint8_t sendMsgBuf(uint32_t id, uint8_t /*ext*/, uint8_t len, const uint8_t* buf) {
        TwaiCanGuard _g(*this);
        serviceBusHealth();
        // 熔斷靜默窗內拒發（回 FAIL）：上層語意皆容忍（F5 每 cycle 重發、讀取保持上次角度）
        if ((int32_t)(busQuietUntilMs - millis()) > 0) return TWAI_CAN_FAIL;
        twai_message_t msg = {};
        msg.identifier = id & 0x7FF;
        msg.extd = 0;
        msg.rtr = 0;
        msg.data_length_code = len > 8 ? 8 : len;
        memcpy(msg.data, buf, msg.data_length_code);
        return (twai_transmit(&msg, 0) == ESP_OK) ? TWAI_CAN_OK : TWAI_CAN_FAIL;
    }

    uint8_t checkReceive() {
        TwaiCanGuard _g(*this);
        serviceBusHealth();
        if (hasCached) return TWAI_CAN_MSGAVAIL;
        if (twai_receive(&cached, 0) == ESP_OK) {
            hasCached = true;
            return TWAI_CAN_MSGAVAIL;
        }
        return TWAI_CAN_NOMSG;
    }

    uint8_t readMsgBuf(unsigned long* id, uint8_t* len, uint8_t* buf) {
        TwaiCanGuard _g(*this);
        if (!hasCached && twai_receive(&cached, 0) != ESP_OK) return TWAI_CAN_FAIL;
        *id = cached.identifier;
        *len = cached.data_length_code;
        memcpy(buf, cached.data, cached.data_length_code);
        hasCached = false;
        return TWAI_CAN_OK;
    }

    uint8_t getEFLG() {
        TwaiCanGuard _g(*this);
        twai_status_info_t st = {};
        if (twai_get_status_info(&st) != ESP_OK) return 0xFF;
        uint8_t flags = 0;
        if (st.state == TWAI_STATE_BUS_OFF) flags |= 0x80;
        if (st.state == TWAI_STATE_RECOVERING) flags |= 0x40;
        if (st.state == TWAI_STATE_STOPPED) flags |= 0x20;
        if (st.rx_missed_count > 0) flags |= 0x10;
        if (st.rx_overrun_count > 0) flags |= 0x08;
        if (st.tx_failed_count > 0) flags |= 0x04;
        return flags;
    }

    uint8_t getTEC() {
        TwaiCanGuard _g(*this);
        twai_status_info_t st = {};
        if (twai_get_status_info(&st) != ESP_OK) return 255;
        return st.tx_error_counter > 255 ? 255 : st.tx_error_counter;
    }

    uint8_t getREC() {
        TwaiCanGuard _g(*this);
        twai_status_info_t st = {};
        if (twai_get_status_info(&st) != ESP_OK) return 255;
        return st.rx_error_counter > 255 ? 255 : st.rx_error_counter;
    }

    uint32_t getRxDropCount() {
        TwaiCanGuard _g(*this);
        twai_status_info_t st = {};
        return (twai_get_status_info(&st) == ESP_OK) ? st.rx_missed_count + st.rx_overrun_count : 0;
    }

    uint32_t getTxFailCount() {
        TwaiCanGuard _g(*this);
        twai_status_info_t st = {};
        return (twai_get_status_info(&st) == ESP_OK) ? st.tx_failed_count : 0;
    }

    uint32_t getRxQueueDepth() {
        TwaiCanGuard _g(*this);
        twai_status_info_t st = {};
        return (twai_get_status_info(&st) == ESP_OK) ? st.msgs_to_rx : 0;
    }

    void clearRXOverflow() {
        // TWAI status counters are cumulative in this ESP-IDF generation; keep
        // this API as a no-op for MCP_CAN compatibility.
    }

    bool setListenOnly() {
        TwaiCanGuard _g(*this);
        if (installed) {
            twai_stop();
            twai_driver_uninstall();
            installed = false;
        }
        twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_PIN, TWAI_RX_PIN, TWAI_MODE_LISTEN_ONLY);
        g.rx_queue_len = TWAI_RX_QUEUE_LEN;
        g.tx_queue_len = TWAI_TX_QUEUE_LEN;
        twai_timing_config_t t = timingFor(bitrate);
        twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
        if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
        installed = true;
        hasCached = false;
        return twai_start() == ESP_OK;
    }

    bool setNormal() {
        return begin(bitrate);
    }

    bool setHardwareFilterToId(uint32_t /*id*/) { return true; }
    bool setHardwareFilterAcceptAll() { return true; }
    bool rawSetHardwareFilter(uint32_t /*targetId*/) { return true; }
    bool rawSetFilterAcceptAll() { return true; }
    uint32_t rawSetHardwareFilterDiag(uint32_t /*targetId*/) { return 3; }
    bool rawSetMode(uint8_t /*modeBits*/) { return true; }
    uint8_t rawReadReg(uint8_t /*addr*/) { return 0; }
    void rawWriteReg(uint8_t /*addr*/, uint8_t /*val*/) {}
};

inline TwaiCanGuard::TwaiCanGuard(TwaiCanCompat& c) : can(c) { can.lock(); }
inline TwaiCanGuard::~TwaiCanGuard() { can.unlock(); }
