# sysid/attic — 已封存的一次性戰役腳本

2026-06-24~26「M4 極限環 / PID 工作點」戰役期間寫的一次性探測腳本，任務已完成
（工作點定案為 vFOC 純P `[1024,0,0,0]` + HOLD 死咬，見 memory `project_pid_working_point`）。
保留供歷史/方法論參考，**不要直接對現行 rig 執行**：

- 多數硬編過時 PID `K 220 30 270 320`（`amp_sweep`/`gate1`/`ki_matrix`/`verify_boot`/`p3_standfree`），
  重跑會把六顆馬達踢離現行純P 工作點，且出口可能留 `Ki=100`（>40 有極限環風險）直到重開機。
- 無 `--live --i-am-at-rig` 安全閘（現行 rig 腳本的標準防誤觸機制），ws 連上即動硬體。
- 依現行 server API：`rec/start`/`rec/stop` 已改要求 POST，這裡的腳本仍用舊 GET → 錄製會失敗（405）。

要重跑任何一支，先移回 `sysid/`、對照現行 `research_trial.js` 補上 preflight 雙閘與工作點 PID。
