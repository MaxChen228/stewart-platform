#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>

#define CAN_CS   5
#define CAN_INT  4
#define NUM_MOTORS 6

MCP_CAN CAN0(CAN_CS);

struct MotorState {
    bool online;
    float encoderDeg;
    unsigned long lastSeen;
    bool spinning;
    unsigned long spinEnd;
};

MotorState motors[NUM_MOTORS];

uint8_t calcCRC(uint8_t id, uint8_t *data, uint8_t len) {
    uint8_t crc = id;
    for (uint8_t i = 0; i < len; i++) crc += data[i];
    return crc & 0xFF;
}

void canSend(uint8_t id, uint8_t *data, uint8_t len) {
    data[len - 1] = calcCRC(id, data, len - 1);
    CAN0.sendMsgBuf(id, 0, len, data);
}

bool canReceive(uint8_t expectId, uint8_t *buf, uint8_t *len, unsigned long timeout = 20) {
    unsigned long start = millis();
    while (millis() - start < timeout) {
        if (CAN_MSGAVAIL == CAN0.checkReceive()) {
            unsigned long rxId;
            CAN0.readMsgBuf(&rxId, len, buf);
            if ((uint8_t)rxId == expectId) return true;
        }
    }
    return false;
}

void drainCAN() {
    while (CAN_MSGAVAIL == CAN0.checkReceive()) {
        uint8_t l; uint8_t b[8]; unsigned long id;
        CAN0.readMsgBuf(&id, &l, b);
    }
}

void enableMotor(uint8_t id, bool en) {
    uint8_t data[3] = {0xF3, (uint8_t)(en ? 0x01 : 0x00), 0x00};
    canSend(id, data, 3);
    uint8_t buf[8]; uint8_t len;
    canReceive(id, buf, &len, 50);
}

void speedMode(uint8_t id, uint16_t speed, uint8_t accel, bool ccw) {
    uint8_t sh = (speed >> 8) & 0x0F;
    if (ccw) sh |= 0x80;
    uint8_t sl = speed & 0xFF;
    uint8_t data[5] = {0xF6, sh, sl, accel, 0x00};
    canSend(id, data, 5);
    uint8_t buf[8]; uint8_t len;
    canReceive(id, buf, &len, 50);
}

void speedStop(uint8_t id) {
    uint8_t data[5] = {0xF6, 0x00, 0x00, 0x05, 0x00};
    canSend(id, data, 5);
    uint8_t buf[8]; uint8_t len;
    canReceive(id, buf, &len, 50);
}

void scanMotors() {
    drainCAN();
    for (int i = 0; i < NUM_MOTORS; i++) {
        uint8_t id = i + 1;
        uint8_t data[2] = {0x30, 0x00};
        canSend(id, data, 2);
        uint8_t buf[8]; uint8_t len;
        if (canReceive(id, buf, &len)) {
            motors[i].online = true;
            motors[i].lastSeen = millis();
            if (buf[0] == 0x30 && len == 8) {
                int32_t carry = ((int32_t)buf[1]<<24) | (buf[2]<<16) | (buf[3]<<8) | buf[4];
                uint16_t val = (buf[5]<<8) | buf[6];
                motors[i].encoderDeg = (carry * 16384.0f + val) * 360.0f / 16384.0f;
            }
        } else {
            if (millis() - motors[i].lastSeen > 1000) motors[i].online = false;
        }
    }
}

void handleSpinTimers() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (motors[i].spinning && millis() >= motors[i].spinEnd) {
            uint8_t id = i + 1;
            speedStop(id);
            delay(50);
            enableMotor(id, false);
            motors[i].spinning = false;
        }
    }
}

void sendStatus() {
    Serial.print("{\"motors\":[");
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (i) Serial.print(",");
        Serial.printf("{\"id\":%d,\"on\":%s,\"deg\":%.1f,\"spin\":%s}",
            i + 1,
            motors[i].online ? "true" : "false",
            motors[i].encoderDeg,
            motors[i].spinning ? "true" : "false");
    }
    Serial.println("]}");
}

void handleSerial() {
    if (!Serial.available()) return;
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("SPIN:")) {
        int id = cmd.substring(5).toInt();
        if (id >= 1 && id <= NUM_MOTORS) {
            int idx = id - 1;
            if (motors[idx].online && !motors[idx].spinning) {
                enableMotor(id, true);
                delay(50);
                speedMode(id, 100, 5, false);
                motors[idx].spinning = true;
                motors[idx].spinEnd = millis() + 3000;
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial);
    delay(500);

    if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
        Serial.println("{\"error\":\"MCP2515 FAILED\"}");
        while (1);
    }
    CAN0.setMode(MCP_NORMAL);
    memset(motors, 0, sizeof(motors));
    Serial.println("{\"ready\":true}");
}

void loop() {
    handleSerial();
    handleSpinTimers();

    static unsigned long lastScan = 0;
    if (millis() - lastScan > 100) {
        scanMotors();
        sendStatus();
        lastScan = millis();
    }
}
