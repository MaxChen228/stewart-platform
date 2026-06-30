# 手機 IMU 控制（陀螺儀模式）

Date: 2026-06-30（取代 2026-06-28 版；舊版描述的「手機頁自跑操作員按鈕」模型已廢）

目標：把手機瀏覽器當平台的即時姿態控制器，**不改韌體運動語意**（仍走 `P` 軌跡 + `PF` 跟隨那套）。

## 架構：owner 仲裁，server 為唯一權威

三個互斥的控制世界，由 server 的 `controlOwner` 仲裁，同時只有一個生效：

| owner | 來源 | 說明 |
|-------|------|------|
| `desktop` | 串流台拖滑桿 | `web/live.html` 拖 6 軸滑桿即時串 PF |
| `phone` | 手機陀螺儀 | `web/phone.html` 傾斜手機串 PF |
| `sim` | 模擬手機 generator | 走真 follow 路徑但**不開 phone-capture**（模擬資料不汙染語料庫） |
| `null` | 無 | 純死咬 / mode 0/1 |

**三來源的唯一入口 = 串流台 `/live.html` 的「來源」三選一**（選來源 → 按「串接板子」→ server 收 `MODE <來源>`）。主控頁 `web/index.html` 已不再有任何串流入口（純定點 P 控制台）。手機頁**自己不送任何模式指令**，只宣告 `{role:'phone'}` 然後等 server 廣播驅動。

`setControlMode`（server.js）切換 owner 時：
- 對韌體送 `FOLLOW 0`（清舊 owner 串流 + 凍結）→ 若有新 owner 再送 `FOLLOW 1`（進新世界）。
- `phone` owner 額外 `phoneCapOpen()` 開 phone-capture 落盤檔；離開時 `phoneCapClose()` 收檔。
- 廣播 `evt:'mode'{owner}` 給所有 client（手機/dashboard 據此同步 UI 與驅動狀態）。

**PF 資料面仲裁**：有 owner 時，server 把「非 owner client 送的 `PF`」一律靜默 drop（`ws._role !== controlOwner`）。手機只有在 `owner==='phone'` 時 PF 才真正下發。未宣告 role 也視同非 owner 擋掉。

**WS origin gate**：只放行 localhost + RFC1918 私網來源（擋 CSWSH / DNS-rebinding 經惡意網頁驅動實體馬達）。LAN 內手機/平板照常可控。

**Session 互斥**：Workspace session 進行中，server 直接拒絕 `MODE`（回報當前 owner，避免三端 desync），手機 PF 也因 owner / session lock 不生效——手動手機控制與自動實驗不爭 `PF`。

## 進入流程（手機模式；2026-06-30 起：先起飛回 home）

1. 電腦開串流台 `/live.html`，「來源」選 **📱 手機** → 彈 QR 配對面板。
2. 手機掃 QR 開 `/phone.html`（HTTPS 才有 `DeviceOrientationEvent` 權限）。WS open → 送 `{role:'phone'}`，顯示「待機 · 等電腦啟用」。
3. 電腦按串流台「串接板子」：
   1. 先跑 `armToHome()` **起飛編排**：依平台當前狀態用 `P` 把平台帶回 home（home/landing 取自 `/api/platform-config`）——
      - `D`（斷電）→ `H`（抓當前下垂、使能死咬）→ `P release`（landing 存在才經此）→ `P home`
      - `H`（死咬中）→ `P home`
      - `E`（使能非死咬）→ 先 `H` 再 `P home`
   2. 到 home 才送 `MODE phone`（server 設 owner=phone、廣播）。
4. 手機收到 `evt:'mode'(owner==='phone')` → `followActive=true` → **自動校正**（平台已在 home，把當前手機姿態設為零點）→ 傾斜手機即驅動。串流台綠 ghost 鏡像手機即時陀螺儀目標（`evt:'followtgt'`），紅 rig=板子實際。
5. 退出：
   - 電腦再按「串接板子」（已串接態）→ `MODE off` → owner 釋放 → 手機 `followActive=false` 停止串流。
   - 或在**主控頁**按 HOME/RELEASE/GO（到達指定位置）→ 一律先 `MODE off` 奪回控制權再走 `P` 軌跡（離散定位一律 P，不走 PF）。

> 拖滑桿（`desktop`）、模擬（`sim`）兩來源的進入流程相同（選來源 → 串接板子 → 起飛 → `MODE <來源>`），差別只在 ghost 意圖來源與是否本頁串 PF。詳見「串流台」段。

## 手機頁職責（web/phone.html）

極簡——**唯一按鈕「校正（歸零當前姿態）」**，其餘全自動 / 由 server 驅動。舊版的 `HOLD HERE`／`RELEASE`／`FOLLOW`／`STOP FOLLOW`／`HOME` 操作員按鈕、以及映射旋鈕（平台角／手機滿刻／Yaw 上限／平滑／死區／軸反向）**全部移除**，移交 dashboard + server 仲裁 + 硬編碼常數。

- **連線**：`ws://${location.host}`（同源，即服務此頁的 HTTPS listener）。on open 送 `{role:'phone'}`。
- **驅動 on/off**：純由 server `evt:'mode'` 驅動 `followActive=(owner==='phone')`，手機端不自決。
- **串流**：`followActive && IK 可達 && !sessionActive` 時，10ms / 100Hz 節流送 `PF x y z r p y`。真實率受手機 `deviceorientation` 事件率封頂（iOS Safari ~60Hz），節流只當地板不創樣本。
- **觀測上傳**：無論是否驅動，`sensorOn` 時恆送 `{obs:'imu', raw, rel, cmd, fol}`（100Hz 上限）→ server 落 phone-capture，供「人手運動 + 感測噪音」源層分析（generator 用；`1€`/predict 是確定性變換可精確重現）。
- **校正**：請求陀螺儀權限 + 把當前手機姿態設為零點；`basePose` 恆 = `homePose`（校正＝平台回 home，傾斜從 home 疊加，不追韌體當前 pose）。進手機模式自動觸發。
- **UI**：姿態圓盤 dot（紅=超工作空間）、roll/pitch/yaw 數值、raw β/γ/α、兩枚 pill（連線 / 待機·手機模式·電腦控制中）。

## 姿態解析與映射（硬編碼，無 UI 旋鈕）

- **解析法**：相對姿態旋轉矩陣（治本，避開 Euler gimbal）。α/β/γ → 旋轉矩陣 `R`（device frame）；校正存 `baseR`；每幀算 `baseRᵀ·curR` 萃取小角 Euler → 三軸解耦、與握持基線無關。yaw 靠 alpha（羅盤）；無羅盤 → yaw 恆 0（誠實不假裝）。
- **映射 `MAP`**：`tiltGain 0.5`（平台度/手機度）、`yawGain 1.4`、`headingDeg 120`（旋轉傾斜向量對齊「手機頭→M1,2 @210°」）、`deadband 0.4°`。線性增益不人為 cap，邊界交給 IK 守界。
- **濾波 `OEF`（One-Euro 自適應）**：取代固定低通——靜止時低 cutoff 殺感測噪音、快動時高 cutoff 低 lag。每軸獨立參數：roll/pitch 低噪 → beta 大反應快；yaw 磁力計高噪 → beta 小保守。`predict 0.08s` 角速度預測抵消下游 ~330ms pipeline lag（VR 頭顯式）；`clamp 4°` 防急反向暫態過衝。
- **工作空間守界**：算出的 pose 不可達 → 維持上一可達 `cmdPose`（dot 標紅、停送該無效目標）。韌體 `followStep` 速限 + `VF` 仍是最終硬安全閘。

## 啟動

`npm start`（= `node server.js`）**同時**起 HTTP dashboard（:3000）與 HTTPS 手機 listener（:3443）——HTTPS 預設開啟，僅 `STEWART_NO_HTTPS=1` 可關（無 openssl/mkcert 的精簡環境）。**沒有獨立的 `start:phone` 指令。**

手機（同 LAN）開：

```text
https://<computer-lan-ip>:3443/phone.html
```

或直接掃串流台 `/live.html`「手機」來源彈出的 QR（推薦——自動帶對 URL，並提供 mkcert 本機 CA 下載）。首次啟動於 `sysid/config/` 建自簽憑證（git ignore）。手機需信任憑證，否則行動瀏覽器會擋 `DeviceOrientationEvent`；裝過 `mkcert -install` 則手機下載安裝一次 rootCA（`http://<lan>:3000/rootCA.pem`）後零警告。

## 串流台 `/live.html`（三來源即時控制中樞）

所有 PF 串流（即時控制）集中在一頁：`npm start` 後開 `http://localhost:3000/live.html`（頁面 `web/live.html`，併入主 :3000，原獨立 `live_server.js :8899` 已退役）。寬敞 3D 與主控頁**完全一致**（紅 rig=板子實際 FK、綠 ghost=當前來源意圖 PF、M1–M6 標籤）+ 工作空間包絡 + 三軸時域波形。

「來源」三選一（互斥，各是一個 server owner）：

| 來源 | owner | ghost 意圖來自 | 板子由誰串 PF |
|------|-------|--------------|--------------|
| 🎯 拖滑桿 | `desktop` | 6 軸相對 home 滑桿 | 本頁拖滑桿（節流 ~100Hz） |
| 📱 手機 | `phone` | server `evt:'followtgt'` 鏡像手機陀螺儀 | phone.html 自串（本頁僅觀察）|
| 🤖 模擬 | `sim` | SSE generator（不開 phone-capture）| 本頁逐幀（×0.1～×500 log 速度）|

- **串接板子鈕**（統一）：先 `armToHome` **起飛回 home**（D/H/E 狀態機，home/landing 取自 `/api/platform-config`，與主控頁同邏輯，避免板子從歪姿直接串 PF 大跳/超界），到位才 `MODE <來源>` 交棒。停止/關頁送 `MODE off` 釋放。串接中鎖來源切換；server `evt:'mode'` 廣播會在被其他端接管時復原本頁。
- **role 仲裁**：PF owner-gate 要求 `ws._role===controlOwner`，故換來源時本頁重新宣告 `{role:<來源>}`。手機來源本頁不送 PF（phone.html role=`phone` 自送），只鏡像 followtgt。
- 跟隨來源（拖滑桿/手機）共用「跟隨緊度（→`FE`）」+「延遲 e2e/傳輸（server `evt:'latency'`）」。
- 後端 lazy：SSE `/stream` 與 `/workspace_data.json` 首次請求才載/算（不拖慢控制 server 啟動）；無 phone-capture 語料庫時 envelope 仍服務、`/stream` 回 503 → 只「模擬」不可用，拖滑桿/手機照常。
- 串流原語在 `gen.js`（`makeBootstrapStream`+`makePhonePipe`，可 `require`，:3000 直接 require）；CLI `node gen.js --dur <sec> --mode bootstrap|iaaft --validate` 寫固定長度檔。

> 詳細生成模型（bootstrap 聯合連續塊 vs iaaft、多場池化、工作空間守界）見記憶 `project_phone_capture_gen`。
