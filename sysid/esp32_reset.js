#!/usr/bin/env node
'use strict';
// ESP32 重啟(免拔線、免燒錄):/api/release 借埠 → pyserial 開埠(預設斷言 RTS 壓住 EN)
// → 釋放 → 晶片重開 → server 自動回連。
// 實測(2026-07-02):node-serialport 的 set() 舞步不可靠(兩次一敗);pyserial「開埠按死」
// 是文檔化行為(1M 戰役的坑反用為 feature),用 platformio penv 的 python。
// 用法:node sysid/esp32_reset.js [--host localhost:3000] [--port /dev/cu.usbserial-0001]
// 成功條件:回連後 uptime 明顯下降(<60s)。exit 0=成功, 1=失敗。
const { execFileSync } = require('child_process');
const os = require('os');
const path = require('path');

const args = process.argv.slice(2);
const opt = (k, d) => { const i = args.indexOf('--' + k); return i >= 0 ? args[i + 1] : d; };
const HOST = opt('host', 'localhost:3000');
const PORT = opt('port', '/dev/cu.usbserial-0001');
const sleep = (ms) => new Promise(r => setTimeout(r, ms));
const latest = () => fetch(`http://${HOST}/api/latest`).then(r => r.json()).catch(() => null);

async function main() {
  const before = await latest();
  const upBefore = before && before.t ? before.t : null;
  await fetch(`http://${HOST}/api/release`, { method: 'POST' }).catch(() => {});
  await sleep(1000);

  // esptool 是唯一每次都成功的重置器(雙電晶體 NAND 自動重置電路:兩線同時斷言=不重置,
  // node-serialport/pyserial 手動 toggle 兩法各實測失敗)。chip_id = 最小 no-op:
  // 進 bootloader 讀 ID → --after hard_reset 回 run mode,全程 ~4s。
  const py = path.join(os.homedir(), '.platformio', 'penv', 'bin', 'python');
  const esptool = path.join(os.homedir(), '.platformio', 'packages', 'tool-esptoolpy', 'esptool.py');
  const out = execFileSync(py, [esptool, '--port', PORT, '--baud', '115200',
    '--after', 'hard_reset', 'chip_id'], { encoding: 'utf8', timeout: 30000 });
  if (!/Hard resetting/i.test(out)) throw new Error('esptool did not hard-reset: ' + out.slice(-200));

  // 等 server 回連 + 遙測恢復(最多 60s)
  const t0 = Date.now();
  while (Date.now() - t0 < 60000) {
    const d = await latest();
    if (d && Number.isFinite(d.t)) {
      if (d.t < 60000 && (upBefore == null || d.t < upBefore)) {
        console.log(JSON.stringify({ reset: true, uptime_s: Math.round(d.t / 1000) }));
        return;
      }
      if (upBefore != null && d.t > upBefore + 120000) break;   // uptime 沒降 = 沒重啟
    }
    await sleep(2000);
  }
  console.error(JSON.stringify({ reset: false, reason: 'uptime did not drop' }));
  process.exit(1);
}
main().catch(e => { console.error(JSON.stringify({ reset: false, reason: e.message })); process.exit(1); });
