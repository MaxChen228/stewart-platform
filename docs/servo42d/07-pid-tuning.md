# PID Parameter Tuning

**Warning: MKS motors have factory-tuned PID. Adjust with extreme caution to avoid motor damage.**

## Three-Loop Control (vFOC mode)

```
Position loop (10kHz): Kp, Ki, Kd → target speed
Speed loop (10kHz):    Kv          → target current
Torque loop (20kHz):               → PWM → motor
```

## 0x96 — vFOC Mode PID

### Set Kp/Ki (DLC=7)

`[96, 00, Kp_hi, Kp_lo, Ki_hi, Ki_lo, CRC]`

- Kp: 0-1024, **default=0xDC (220)**
- Ki: 0-1024, **default=0x64 (100)**

### Set Kd/Kv (DLC=7)

`[96, 01, Kd_hi, Kd_lo, Kv_hi, Kv_lo, CRC]`

- Kd: 0-1024, **default=0x10E (270)**
- Kv: 0-1024, **default=0x140 (320)**

Response DLC=3: `[96, status, CRC]`

### Read vFOC PID

- Read Kp/Ki: send `[00, 96, 00, CRC]` (via 0x00 read config)
- Read Kd/Kv: send `[00, 96, 01, CRC]`

## 0x97 — CLOSE Mode PID

### Set Kp/Ki (DLC=7)

`[97, 00, Kp_hi, Kp_lo, Ki_hi, Ki_lo, CRC]`

- Kp: 0-1024, **default=0xC8 (200)**
- Ki: 0-1024, **default=0x50 (80)**

### Set Kd/Kv (DLC=7)

`[97, 01, Kd_hi, Kd_lo, Kv_hi, Kv_lo, CRC]`

- Kd: 0-1024, **default=0xFA (250)**
- Kv: 0-1024, **default=0x12C (300)**

Response DLC=3: `[97, status, CRC]`

### Read CLOSE PID

- Read Kp/Ki: send `[00, 97, 00, CRC]`
- Read Kd/Kv: send `[00, 97, 01, CRC]`

## Default Summary

| Param | vFOC | CLOSE |
|-------|------|-------|
| Kp | 220 | 200 |
| Ki | 100 | 80 |
| Kd | 270 | 250 |
| Kv | 320 | 300 |
