# MKS SERVO42D CAN 測試專案

## 硬體配置
- ESP32 DevKit V1
- MCP2515 CAN 模組（5V 供電）
- MKS SERVO42D 閉環步進馬達（24V 供電）

## SPI 接線（ESP32 → MCP2515）
| ESP32 | MCP2515 |
|-------|---------|
| GPIO 18 | SCK |
| GPIO 23 | MOSI |
| GPIO 19 | MISO |
| GPIO 5 | CS |
| GPIO 4 | INT |
| VIN (5V) | VCC |
| GND | GND |

## CAN 接線（MCP2515 → SERVO42D）
| MCP2515 | SERVO42D |
|---------|----------|
| H | CAN_H |
| L | CAN_L |

## 共地
ESP32 GND = MCP2515 GND = SERVO42D GND（三者共地）

## SERVO42D 設定
- Mode: SR_vFOC
- CAN ID: 1
- CAN Rate: 500K
- 已完成校準（CAL）

## CAN 協議重點
- 標準 CAN 2.0A（11-bit ID）
- CAN ID = 馬達地址（預設 1）
- 資料格式：[功能碼, 資料..., CRC]
- CRC = 所有 data bytes 加總 & 0xFF

## 常用指令
| 功能碼 | 用途 | 資料格式 |
|--------|------|---------|
| 0x30 | 讀編碼器 | [0x30, CRC] |
| 0x32 | 讀轉速 | [0x32, CRC] |
| 0xF3 | 使能/失能 | [0xF3, 0x01=使能/0x00=失能, CRC] |
| 0xF6 | 速度模式 | [0xF6, dir+speed_H, speed_L, accel, CRC]，dir: bit7=1 為 CCW |
| 0xF7 | 緊急停止 | [0xF7, CRC] |
| 0xFD | 相對位置 | [0xFD, dir+speed_H, speed_L, accel, pulses_H, pulses_M, pulses_L, CRC] |

## 目標
1. 建立 PlatformIO 專案（Arduino framework）
2. 使用 mcp_can 程式庫與 SERVO42D 通訊
3. 實作基本功能：讀編碼器、使能馬達、速度模式正轉/反轉、停止
4. Serial Monitor 顯示回傳資料
