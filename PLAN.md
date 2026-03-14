# 實作計劃

## 階段一：基礎建設

### 1.1 PlatformIO 專案配置
- `platformio.ini`：ESP32 board、framework、序列埠速率
- 依賴：`mcp_can`（Cory Fowler）或直接用 SPI 操作 MCP2515

### 1.2 CAN 通訊層 (`can_bus.h/cpp`)
- MCP2515 初始化（500Kbps，SPI 腳位）
- `can_send(id, data[], len)` — 發送幀，自動算 CRC
- `can_read(id*, data[], len*)` — 接收幀
- CRC 校驗：`(CAN_ID + data bytes) & 0xFF`

### 1.3 SERVO42D 驅動層 (`servo42d.h/cpp`)
- `servo_read_encoder(id)` → 回傳角度（轉換 14-bit 原始值）
- `servo_set_speed(id, direction, speed, accel)` — 0xF6 速度指令
- `servo_enable(id, on/off)` — 0xF3 使能
- `servo_stop(id)` — 0xF7 緊急停止
- `servo_read_status(id)` — 0xF1 狀態查詢

---

## 階段二：逆向運動學

### 2.1 幾何參數定義 (`kinematics.h`)
- 底座附著點 B[6]：半徑 + 角度陣列
- 平台附著點 P[6]：半徑 + 角度陣列
- 下腿長 (lower_leg)、上腿長 (upper_leg)
- 馬達平面角 motor_plane_angle[6]
- **以上參數需量測實際機構後填入**

### 2.2 逆向運動學求解 (`kinematics.cpp`)
- 輸入：`float x, y, z, roll, pitch, yaw`
- 輸出：`float angles[6]`（六個馬達目標角度）
- 旋轉矩陣 R = Rz × Ry × Rx
- 對每條腿解 `asin(L/√(M²+N²)) - atan2(N,M)`
- 回傳 valid/invalid（角度為實數 = 可達）

### 2.3 角度↔編碼器換算
- 目標角度 → 目標編碼器值（用於 PID 比較）
- 編碼器原始值 → 實際角度（14-bit, 0~16383 = 0~360°）
- 需處理零點偏移（機構安裝時的初始角度）

---

## 階段三：集中式控制迴圈

### 3.1 PID 控制器 (`controller.h/cpp`)
- 六軸共用一組 Kp/Ki/Kd 參數（初期）
- 每個控制週期（10~20ms）：
  1. 讀 6 顆編碼器 (0x30 × 6)
  2. 逆向運動學算出 6 個目標角度
  3. 計算誤差 = target - actual
  4. PID 算出速度指令
  5. 發 6 個速度指令 (0xF6 × 6)
- 輸出限幅：速度 0~3000 RPM，方向自動判斷

### 3.2 安全機制
- 角度誤差超過閾值 → 緊急停止所有馬達
- CAN 通訊逾時 → 停止
- 序列埠指令可手動觸發停止

---

## 階段四：使用者介面

### 4.1 序列埠指令（最低限度）
- `M x y z roll pitch yaw` — 移動到指定姿態
- `H` — 回到中立位置
- `S` — 緊急停止
- `P kp ki kd` — 調整 PID 參數
- `E` — 使能/禁用所有馬達

### 4.2 Web UI（後續）
- ESP32 AsyncWebServer + WebSocket
- 即時 6DOF 滑桿控制
- 顯示編碼器回饋、馬達狀態

---

## 階段五：調參與優化

### 5.1 PID 調參流程
1. Ki=0, Kd=0, Kp 從極小值開始（~0.01）
2. 確認不發散後逐步加大 Kp
3. 有穩態誤差 → 加小量 Ki
4. 有過衝 → 加 Kd
5. 先求穩，再求快

### 5.2 CAN 鮑率升級
- 確認穩定後用 0x8A 將六顆馬達升至 1Mbps
- MCP2515 初始化改為 1Mbps
- 控制頻率可從 ~200Hz 提升至 ~400Hz

---

## 檔案結構（預計）

```
stewart-platform/
├── CLAUDE.md
├── PLAN.md
├── platformio.ini
├── src/
│   ├── main.cpp            # 主程式、控制迴圈
│   ├── can_bus.h / .cpp     # MCP2515 CAN 通訊
│   ├── servo42d.h / .cpp    # SERVO42D 驅動指令
│   ├── kinematics.h / .cpp  # 逆向運動學
│   └── controller.h / .cpp  # PID 控制器
└── web/
    └── index.html           # Web UI（階段四）
```

---

## 實作順序

```
1.1 → 1.2 → 1.3 → 單軸通訊驗證
                  ↓
              2.1 → 2.2 → 2.3 → 數值驗證（對照 MATLAB 結果）
                              ↓
                          3.1 → 3.2 → 單軸閉環測試 → 六軸閉環
                                                      ↓
                                                  4.1 → 4.2 → 5.1 → 5.2
```

每個階段完成後都要在實機上驗證再進下一階段。
