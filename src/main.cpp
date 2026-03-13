#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>

#define CAN_CS   5
#define CAN_INT  4
#define NUM_MOTORS 6
#define SERVO_PULSES_PER_REV 6400
#define POSITION_SPEED 180
#define POSITION_ACCEL 12

MCP_CAN CAN0(CAN_CS);

struct MotorState {
    bool online;
    float rawEncoderDeg;
    float encoderDeg;
    float targetDeg;
    float zeroOffsetDeg;
    unsigned long lastSeen;
    bool enabled;
    bool moving;
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
    motors[id - 1].enabled = en;
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

void emergencyStop(uint8_t id) {
    uint8_t data[2] = {0xF7, 0x00};
    canSend(id, data, 2);
    uint8_t buf[8]; uint8_t len;
    canReceive(id, buf, &len, 50);
    motors[id - 1].moving = false;
}

void zeroMotor(uint8_t id) {
    int index = id - 1;
    motors[index].zeroOffsetDeg = motors[index].rawEncoderDeg;
    motors[index].encoderDeg = 0.0f;
    motors[index].targetDeg = 0.0f;
    motors[index].moving = false;
}

void positionMode(uint8_t id, float deltaDeg) {
    uint32_t pulses = (uint32_t)lroundf(fabsf(deltaDeg) * SERVO_PULSES_PER_REV / 360.0f);
    if (pulses == 0) {
        motors[id - 1].moving = false;
        return;
    }

    uint8_t sh = (POSITION_SPEED >> 8) & 0x0F;
    if (deltaDeg < 0) sh |= 0x80;
    uint8_t sl = POSITION_SPEED & 0xFF;

    uint8_t data[8] = {
        0xFD,
        sh,
        sl,
        POSITION_ACCEL,
        (uint8_t)((pulses >> 16) & 0xFF),
        (uint8_t)((pulses >> 8) & 0xFF),
        (uint8_t)(pulses & 0xFF),
        0x00
    };
    canSend(id, data, 8);
    uint8_t buf[8]; uint8_t len;
    canReceive(id, buf, &len, 50);
    motors[id - 1].moving = true;
}

void scanMotors() {
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
                motors[i].rawEncoderDeg = (carry * 16384.0f + val) * 360.0f / 16384.0f;
                motors[i].encoderDeg = motors[i].rawEncoderDeg - motors[i].zeroOffsetDeg;
                if (fabsf(motors[i].targetDeg - motors[i].encoderDeg) < 1.0f) {
                    motors[i].moving = false;
                }
            }
        } else if (millis() - motors[i].lastSeen > 1000) {
            motors[i].online = false;
            motors[i].moving = false;
        }
    }
}

void sendStatus() {
    Serial.print("{\"motors\":[");
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (i) Serial.print(",");
        Serial.printf("{\"id\":%d,\"on\":%s,\"deg\":%.1f,\"rawDeg\":%.1f,\"targetDeg\":%.1f,\"enabled\":%s,\"moving\":%s}",
            i + 1,
            motors[i].online ? "true" : "false",
            motors[i].encoderDeg,
            motors[i].rawEncoderDeg,
            motors[i].targetDeg,
            motors[i].enabled ? "true" : "false",
            motors[i].moving ? "true" : "false");
    }
    Serial.println("]}");
}

void handleSerial() {
    if (!Serial.available()) return;
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("ENABLE:")) {
        bool enable = cmd.substring(7).toInt() == 1;
        for (int i = 0; i < NUM_MOTORS; i++) {
            enableMotor(i + 1, enable);
            delay(10);
        }
    } else if (cmd == "STOP") {
        for (int i = 0; i < NUM_MOTORS; i++) {
            emergencyStop(i + 1);
            delay(10);
        }
    } else if (cmd.startsWith("MOVE:")) {
        int start = 5;
        for (int i = 0; i < NUM_MOTORS; i++) {
            int comma = cmd.indexOf(',', start);
            String token = comma == -1 ? cmd.substring(start) : cmd.substring(start, comma);
            float nextTarget = token.toFloat();
            float delta = nextTarget - motors[i].targetDeg;
            if (!motors[i].enabled) {
                enableMotor(i + 1, true);
                delay(10);
            }
            positionMode(i + 1, delta);
            motors[i].targetDeg = nextTarget;
            start = comma == -1 ? cmd.length() : comma + 1;
        }
    } else if (cmd.startsWith("ZERO:")) {
        int id = cmd.substring(5).toInt();
        if (id >= 1 && id <= NUM_MOTORS) {
            zeroMotor(id);
        }
    } else if (cmd.startsWith("SPIN:")) {
        int id = cmd.substring(5).toInt();
        if (id >= 1 && id <= NUM_MOTORS) {
            enableMotor(id, true);
            delay(20);
            speedMode(id, 100, 5, false);
            motors[id - 1].moving = true;
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

    static unsigned long lastScan = 0;
    if (millis() - lastScan > 100) {
        drainCAN();
        scanMotors();
        sendStatus();
        lastScan = millis();
    }
}
