#!/usr/bin/env node
'use strict';
// ===== Live phone-motion generator server（即時/無限串流）=====
// 複用 gen.js 的串流原語（makeBootstrapStream + makePhonePipe）以真實 60Hz wall-clock 無限生成 PF pose，
// 經 SSE 推給 live.html → 共用 stewart-renderer 的 3D 模型即時動 + 軌跡冒點。零外部依賴（純 http + SSE）。
//
// 用法：node sysid/phone_gen/live_server.js [port]  （預設 8899）。瀏覽器開 http://localhost:<port>/
const http = require('http');
const fs = require('fs');
const path = require('path');
const G = require('./gen.js');

const PORT = Number(process.argv[2] || 8899);
const ROOT = __dirname;                              // sysid/phone_gen/
const PROJ = path.join(__dirname, '..', '..');       // stewart-platform/
const SHARED = path.join(PROJ, 'web', 'shared');

const { computeWorkspaceData } = require('./workspace_viz.js');

const sessions = G.loadSessions(G.defaultRefFiles());
if (!sessions.length) { console.error('無可用參考 capture（sysid/data/phone-capture/ 需 ≥1 場、每場 ≥2048 imu）'); process.exit(1); }
const WS_DATA = computeWorkspaceData();   // 工作空間包絡+橘歷史，啟動算一次（kin.js 掃網格 ~數萬次 IK）→ serve /workspace_data.json
const WS_JSON = JSON.stringify(WS_DATA);

const MIME = { '.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8', '.css': 'text/css', '.json': 'application/json' };
const r3 = v => Math.round(v * 1000) / 1000;

function serveFile(res, file, sandbox) {
  if (sandbox && file !== sandbox && !file.startsWith(sandbox + path.sep)) { res.writeHead(403); res.end('forbidden'); return; }
  fs.readFile(file, (e, d) => {
    if (e) { res.writeHead(404); res.end('not found'); return; }
    res.writeHead(200, { 'Content-Type': MIME[path.extname(file)] || 'text/plain', 'Cache-Control': 'no-store' });
    res.end(d);
  });
}

// SSE：每連線一條獨立 stream（自身 rng + 1€濾波器狀態）→ 訊息率固定 60/s。speed＝每 tick 平均產出樣本數，
// 浮點（0.05~500）：>1 壓縮時間軸（每 tick 多點，秒級累積整片分布）；<1 慢動作（分數累加器，某些 tick 不足一點就不送，
// 真「一點一點冒」）。client 關閉即停。
function startStream(req, res, u) {
  const seed = Number(u.searchParams.get('seed')) || Math.floor(Math.random() * 1e9);
  const speed = Math.max(0.05, Math.min(500, Number(u.searchParams.get('speed')) || 1));
  const rng = G.mulberry32(seed);
  const stream = G.makeBootstrapStream(sessions, rng);
  const pipe = G.makePhonePipe();
  res.writeHead(200, {
    'Content-Type': 'text/event-stream',
    'Cache-Control': 'no-store',
    'Connection': 'keep-alive',
    'X-Accel-Buffering': 'no',
  });
  res.write(`event: meta\ndata: ${JSON.stringify({ seed, rate: G.RATE, speed, sessions: sessions.length, home: G.HOME })}\n\n`);
  let n = 0, ikDrop = 0, acc = 0;
  const iv = setInterval(() => {
    acc += speed;
    const count = Math.floor(acc); acc -= count;            // 分數累加：慢速時 count 可為 0
    if (count <= 0) return;                                 // 本 tick 不足一點 → 不送（保持連線）
    const pts = new Array(count); let last = null;
    for (let b = 0; b < count; b++) {
      const rel = stream.next();
      const { pose, valid } = pipe.step(rel);
      if (!valid) ikDrop++;
      pts[b] = [r3(pose[3]), r3(pose[4]), r3(pose[5])];     // 累積用：每樣本 (roll,pitch,yaw)
      last = pose; n++;
    }
    res.write(`data: ${JSON.stringify({ n, ikDrop, last: last.map(r3), pts })}\n\n`);   // last 全 pose 給 rig，pts 整批給累積
  }, 1000 / G.RATE);
  req.on('close', () => clearInterval(iv));
}

const server = http.createServer((req, res) => {
  const u = new URL(req.url, 'http://localhost');
  const p = u.pathname;
  if (p === '/' || p === '/live.html') return serveFile(res, path.join(ROOT, 'live.html'), ROOT);   // 合併頁：左工作空間即時累積 × 右 rig
  if (p === '/workspace_data.json') { res.writeHead(200, { 'Content-Type': 'application/json', 'Cache-Control': 'no-store' }); return res.end(WS_JSON); }
  if (p.endsWith('.html')) return serveFile(res, path.join(ROOT, path.basename(p)), ROOT);   // phone_gen 其餘產出頁（沙箱：僅檔名）
  if (p === '/sysid/kin.js') return serveFile(res, path.join(PROJ, 'sysid', 'kin.js'));
  if (p.startsWith('/shared/')) return serveFile(res, path.join(SHARED, p.slice('/shared/'.length)), SHARED);   // app-ui.css + 共用 .js（含 css，沙箱）
  if (p === '/stream') return startStream(req, res, u);
  res.writeHead(404); res.end('not found');
});
server.listen(PORT, () => {
  console.log(`live-gen :${PORT}  參考 ${sessions.length} 場 (lens ${sessions.map(s => s.len).join(',')})  → http://localhost:${PORT}/`);
});
