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

    void ensureMutex() {
        if (!canMux) canMux = xSemaphoreCreateRecursiveMutex();
    }

    static twai_timing_config_t timingFor(uint32_t bps) {
        switch (bps) {
            case 1000000: return TWAI_TIMING_CONFIG_1MBITS();
            case 500000:  return TWAI_TIMING_CONFIG_500KBITS();
            case 250000:  return TWAI_TIMING_CONFIG_250KBITS();
            case 125000:  return TWAI_TIMING_CONFIG_125KBITS();
            default:      return TWAI_TIMING_CONFIG_500KBITS();
        }
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
