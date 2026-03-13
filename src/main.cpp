#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>

#define CAN_CS   5
#define CAN_INT  4
#define NUM_MOTORS 6
#define SERVO_PULSES_PER_REV 3200
#define POSITION_SPEED 180
#define POSITION_ACCEL 12
#define TELEMETRY_INTERVAL_MS 50
#define MOTOR_SCAN_INTERVAL_MS 8
#define CONFIG_SCAN_INTERVAL_MS 80
#define MOTOR_SCAN_TIMEOUT_MS 6
#define CONFIG_SCAN_TIMEOUT_MS 8

MCP_CAN CAN0(CAN_CS);

enum ConfigCode : uint8_t {
    CONFIG_MODE = 0x82,
    CONFIG_WORK_CURRENT = 0x83,
    CONFIG_HOLD_CURRENT = 0x9B,
};

enum ConfigFlag : uint8_t {
    CONFIG_FLAG_MODE = 0x01,
    CONFIG_FLAG_WORK_CURRENT = 0x02,
    CONFIG_FLAG_HOLD_CURRENT = 0x04,
    CONFIG_FLAG_ALL = CONFIG_FLAG_MODE | CONFIG_FLAG_WORK_CURRENT | CONFIG_FLAG_HOLD_CURRENT,
};

struct MotorState {
    bool online;
    int32_t rawEncoderCount;
    uint16_t singleTurnCount;
    float rawEncoderDeg;
    float encoderDeg;
    float targetDeg;
    float zeroOffsetDeg;
    uint32_t zeroSeq;
    unsigned long lastSeen;
    bool enabled;
    bool moving;
    uint8_t modeCode;
    uint16_t workCurrentMa;
    uint8_t holdCurrentRatio;
    bool configKnown;
    uint8_t configMask;
    unsigned long lastConfigSeen;
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

const char *modeName(uint8_t modeCode) {
    switch (modeCode) {
        case 0x00: return "CR_OPEN";
        case 0x01: return "CR_CLOSE";
        case 0x02: return "CR_vFOC";
        case 0x03: return "SR_OPEN";
        case 0x04: return "SR_CLOSE";
        case 0x05: return "SR_vFOC";
        default: return "UNKNOWN";
    }
}

void updateConfigKnown(int index) {
    motors[index].configKnown = motors[index].configMask == CONFIG_FLAG_ALL;
}

void clearMotorConfig(int index) {
    motors[index].modeCode = 0;
    motors[index].workCurrentMa = 0;
    motors[index].holdCurrentRatio = 0;
    motors[index].configKnown = false;
    motors[index].configMask = 0;
    motors[index].lastConfigSeen = 0;
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
    motors[index].zeroSeq++;
}

void calibrateAllMotors() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        zeroMotor(i + 1);
    }
}

void positionMode(uint8_t id, float deltaDeg) {
    uint32_t pulses = (uint32_t)lroundf(fabsf(deltaDeg) * SERVO_PULSES_PER_REV / 360.0f);
    if (pulses == 0) {
        motors[id - 1].moving = false;
        return;
    }

    uint8_t sh = (POSITION_SPEED >> 8) & 0x0F;
    // The observed hardware motion is inverted relative to the vendor examples
    // we modeled from, so the relative-position direction bit must be flipped
    // to keep commanded servo angles aligned with measured encoder angles.
    if (deltaDeg > 0) sh |= 0x80;
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

bool readConfigParam(uint8_t id, uint8_t code, uint8_t *buf, uint8_t *len, unsigned long timeout = CONFIG_SCAN_TIMEOUT_MS) {
    uint8_t data[3] = {0x00, code, 0x00};
    canSend(id, data, 3);
    if (!canReceive(id, buf, len, timeout)) {
        return false;
    }
    return *len >= 3 && buf[0] == code;
}

void scanMotor(uint8_t id) {
    int index = id - 1;
    uint8_t data[2] = {0x30, 0x00};
    canSend(id, data, 2);
    uint8_t buf[8]; uint8_t len;
    if (canReceive(id, buf, &len, MOTOR_SCAN_TIMEOUT_MS)) {
        motors[index].online = true;
        motors[index].lastSeen = millis();
        if (buf[0] == 0x30 && len == 8) {
            int32_t carry = ((int32_t)buf[1]<<24) | (buf[2]<<16) | (buf[3]<<8) | buf[4];
            uint16_t val = (buf[5]<<8) | buf[6];
            motors[index].rawEncoderCount = carry * 16384 + val;
            motors[index].singleTurnCount = val;
            motors[index].rawEncoderDeg = (carry * 16384.0f + val) * 360.0f / 16384.0f;
            motors[index].encoderDeg = motors[index].rawEncoderDeg - motors[index].zeroOffsetDeg;
            if (fabsf(motors[index].targetDeg - motors[index].encoderDeg) < 1.0f) {
                motors[index].moving = false;
            }
        }
    } else if (millis() - motors[index].lastSeen > 1000) {
        motors[index].online = false;
        motors[index].moving = false;
        clearMotorConfig(index);
    }
}

void scanOneConfig(uint8_t id, uint8_t code) {
    int index = id - 1;
    uint8_t buf[8]; uint8_t len = 0;
    if (!readConfigParam(id, code, buf, &len)) {
        return;
    }

    if (len == 4 && buf[1] == 0xFF && buf[2] == 0xFF) {
        return;
    }

    if (code == CONFIG_MODE && len >= 3) {
        motors[index].modeCode = buf[1];
        motors[index].configMask |= CONFIG_FLAG_MODE;
        motors[index].lastConfigSeen = millis();
    } else if (code == CONFIG_WORK_CURRENT && len >= 4) {
        motors[index].workCurrentMa = (uint16_t)(buf[1] << 8) | buf[2];
        motors[index].configMask |= CONFIG_FLAG_WORK_CURRENT;
        motors[index].lastConfigSeen = millis();
    } else if (code == CONFIG_HOLD_CURRENT && len >= 3) {
        motors[index].holdCurrentRatio = buf[1];
        motors[index].configMask |= CONFIG_FLAG_HOLD_CURRENT;
        motors[index].lastConfigSeen = millis();
    }
    updateConfigKnown(index);
}

void sendStatus() {
    Serial.print("{\"motors\":[");
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (i) Serial.print(",");
        int holdPct = motors[i].configKnown ? ((int)motors[i].holdCurrentRatio * 10 + 10) : 0;
        Serial.printf("{\"id\":%d,\"on\":%s,\"deg\":%.1f,\"rawDeg\":%.1f,\"encoderCount\":%ld,\"singleTurnCount\":%u,\"targetDeg\":%.1f,\"zeroOffsetDeg\":%.1f,\"zeroSeq\":%lu,\"enabled\":%s,\"moving\":%s,\"modeCode\":%u,\"mode\":\"%s\",\"workCurrentMa\":%u,\"holdCurrentRatio\":%u,\"holdCurrentPct\":%d,\"configKnown\":%s}",
            i + 1,
            motors[i].online ? "true" : "false",
            motors[i].encoderDeg,
            motors[i].rawEncoderDeg,
            (long)motors[i].rawEncoderCount,
            motors[i].singleTurnCount,
            motors[i].targetDeg,
            motors[i].zeroOffsetDeg,
            (unsigned long)motors[i].zeroSeq,
            motors[i].enabled ? "true" : "false",
            motors[i].moving ? "true" : "false",
            motors[i].modeCode,
            modeName(motors[i].modeCode),
            motors[i].workCurrentMa,
            motors[i].holdCurrentRatio,
            holdPct,
            motors[i].configKnown ? "true" : "false");
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
            // Relative position mode must be referenced from the measured actuator angle,
            // otherwise any missed motion or manual movement accumulates permanent drift.
            float delta = nextTarget - motors[i].encoderDeg;
            if (!motors[i].enabled) {
                enableMotor(i + 1, true);
                delay(10);
            }
            positionMode(i + 1, delta);
            motors[i].targetDeg = nextTarget;
            start = comma == -1 ? cmd.length() : comma + 1;
        }
    } else if (cmd == "CALIBRATE") {
        calibrateAllMotors();
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

    static unsigned long lastTelemetry = 0;
    static unsigned long lastMotorScan = 0;
    static unsigned long lastConfigScan = 0;
    static uint8_t nextMotor = 1;
    static uint8_t nextConfigMotor = 1;
    static uint8_t nextConfigIndex = 0;
    static const uint8_t configCodes[] = {CONFIG_MODE, CONFIG_WORK_CURRENT, CONFIG_HOLD_CURRENT};

    if (millis() - lastMotorScan >= MOTOR_SCAN_INTERVAL_MS) {
        drainCAN();
        scanMotor(nextMotor);
        nextMotor = nextMotor >= NUM_MOTORS ? 1 : nextMotor + 1;
        lastMotorScan = millis();
    }

    if (millis() - lastConfigScan >= CONFIG_SCAN_INTERVAL_MS) {
        drainCAN();
        scanOneConfig(nextConfigMotor, configCodes[nextConfigIndex]);
        nextConfigIndex++;
        if (nextConfigIndex >= sizeof(configCodes)) {
            nextConfigIndex = 0;
            nextConfigMotor = nextConfigMotor >= NUM_MOTORS ? 1 : nextConfigMotor + 1;
        }
        lastConfigScan = millis();
    }

    if (millis() - lastTelemetry >= TELEMETRY_INTERVAL_MS) {
        sendStatus();
        lastTelemetry = millis();
    }
}
