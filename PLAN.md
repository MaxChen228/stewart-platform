# 當前進度與下一步

## 現狀

平台**未達穩定平衡**。輕微擾動下追蹤可運作，稍大擾動即發散且不會自行恢復。
IK / FK / CAN / 編碼器 / 序列協議 / Web UI 的基礎設施跑得起來，但整體閉環穩定性沒到可宣稱「完成」的程度。

## 已寫好但未驗證穩定的部分

- ESP32 韌體：CAN、SERVO42D 驅動、編碼器、NVS 校正
- IK 與距離約束 Newton-Raphson FK
- Joint-space 自適應追蹤（mode 0）
- Task-space PD（mode 1，預設）+ FK 失敗 fallback
- Node.js WebSocket 中繼 + Web UI

「寫好」≠「調穩」。以下都還需逐一驗證。

## 下一步

1. **單軸方向交叉驗證**：T0~T5 + 六軸同時微動，確認 `MOTOR_SIGN` 與 `angleToCoord` 在耦合下也正確
2. **找穩定點**：
   - 先把 controlMode 切回 0，trackingMu 拉高、maxGain 壓低，找一組「擾動下不發散」的參數
   - 確認 joint-space 能撐住擾動後，再開 mode 1 並從極低 Kp 重新掃
3. **隔離震盪源**：
   - 把 posSpeed 從 30 降到 5、posAcc 從 5 降到 2，看震盪是否消失
   - 若降速能穩 → 馬達內部 vFOC PID 過硬，調 0x96
   - 若降速也震 → 是 ESP32 外環增益或 FK 雜訊問題
4. **不要先調精度**：穩定收斂之前，任何 ±X° 的目標都沒意義

## 策略

- 先求不發散，再求收斂，最後才談精度
- 每次只動一個變數，並記錄擾動下的行為（不是穩態誤差）
- maxError 急停與工作空間限制保留，不為了「看起來會動」放寬安全機制
