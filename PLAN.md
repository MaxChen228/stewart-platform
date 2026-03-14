# 當前進度與下一步

## 已完成 ✓

- ESP32 韌體：CAN 通訊、SERVO42D 驅動、編碼器讀取
- IK (逆向運動學)：pose → 6 motor angles
- FK (正向運動學)：距離約束 Newton-Raphson，解析 Jacobian
- NVS 校正持久化
- Node.js WebSocket 中繼 server
- Web UI：3D 視覺化 + PID 控制面板
- PID 框架 + 安全機制
- 幾何 CW 排列修正

## 進行中 — 六軸 PID 調參

### 立即要做

1. **燒錄 CW 修正後的韌體 + 重新 Zero All**
2. **單軸驗證**：逐一 enable 單顆馬達，確認方向正確
3. **六軸 PID 調參**：
   - P-only 開始 (Kp=3, Ki=0, Kd=0)
   - 確認六軸全部正確收斂後再加 Kd
   - 目標：穩定收斂到 ±1° 以內

### 調參策略

- 先求穩，再求快
- maxRPM=10 限速，確保安全
- 死區 1.5° 防低速抖動
- maxError=20° 急停保護
