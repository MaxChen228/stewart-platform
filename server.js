const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');
const { WebSocketServer } = require('ws');
const http = require('http');
const fs = require('fs');
const path = require('path');

const HTTP_PORT = 3000;
const BAUD = 115200;
// 綁定位址：預設 0.0.0.0（方便站在機台旁用手機/平板驅動平台）。
// 這是會動的實體機械——若在不可信網段，設環境變數 LOOPBACK_ONLY=1 只綁本機。
const BIND_HOST = process.env.LOOPBACK_ONLY === '1' ? '127.0.0.1' : '0.0.0.0';

// 最新一筆資料（供 REST API 用）
let lastData = null;

// ===== Phase 0 系統辨識：資料記錄器 =====
// 把每筆進來的遙測（dir:"in"）與每筆送出的指令（dir:"cmd"）寫成 JSONL，
// 時間戳用 performance.now()（單調、sub-ms），供離線分析對齊。
const REC_DIR = path.join(__dirname, 'sysid', 'data');
let recStream = null;
let recPath = null;
let recCount = 0;
let recT0 = 0;

function recWrite(dir, payload) {
  if (!recStream) return;
  recStream.write(JSON.stringify({ t: performance.now() - recT0, dir, d: payload }) + '\n');
  recCount++;
}

function recStart(name) {
  if (recStream) recStop();
  fs.mkdirSync(REC_DIR, { recursive: true });
  const safe = (name || 'capture').replace(/[^a-zA-Z0-9_-]/g, '_');
  const stamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
  recPath = path.join(REC_DIR, `${safe}_${stamp}.jsonl`);
  recStream = fs.createWriteStream(recPath);
  recT0 = performance.now();
  recCount = 0;
  // 第一行寫 meta（牆鐘時間 + 標籤），方便事後辨識
  recStream.write(JSON.stringify({ meta: { name: safe, wallClock: new Date().toISOString(), baud: BAUD } }) + '\n');
  console.log(`[Rec] started: ${recPath}`);
  return recPath;
}

function recStop() {
  if (!recStream) return null;
  const p = recPath, n = recCount;
  recStream.end();
  recStream = null;
  console.log(`[Rec] stopped: ${p} (${n} lines)`);
  return { path: p, lines: n };
}

// ===== HTTP 靜態檔案伺服器 =====
const MIME = { '.html':'text/html', '.js':'application/javascript', '.css':'text/css', '.json':'application/json' };

const server = http.createServer((req, res) => {
  // REST: 最新資料
  if (req.url === '/api/latest') {
    res.writeHead(200, { 'Content-Type': 'application/json' });
    res.end(lastData || '{}');
    return;
  }

  // 系統辨識記錄器控制
  if (req.url.startsWith('/api/rec/start')) {
    const name = new URL(req.url, 'http://x').searchParams.get('name');
    const p = recStart(name);
    res.writeHead(200, { 'Content-Type': 'application/json' });
    res.end(JSON.stringify({ recording: true, path: p }));
    return;
  }
  if (req.url === '/api/rec/stop') {
    const r = recStop();
    res.writeHead(200, { 'Content-Type': 'application/json' });
    res.end(JSON.stringify({ recording: false, ...(r || {}) }));
    return;
  }
  if (req.url === '/api/rec/status') {
    res.writeHead(200, { 'Content-Type': 'application/json' });
    res.end(JSON.stringify({ recording: !!recStream, path: recPath, lines: recCount }));
    return;
  }

  // 釋放序列埠（供韌體上傳用，暫停 30 秒重連）
  if (req.url === '/api/release') {
    uploadMode = true;
    if (serial && serial.isOpen) {
      serial.close();
      console.log('[Serial] released for upload (30s hold)');
    }
    setTimeout(() => { uploadMode = false; connectSerial(); }, 30000);
    res.writeHead(200, { 'Content-Type': 'application/json' });
    res.end('{"released":true}');
    return;
  }

  // 靜態檔案（去查詢字串；目錄/結尾 → index.html）
  let urlPath = req.url.split('?')[0];
  if (urlPath === '/') urlPath = '/index.html';
  else if (urlPath.endsWith('/')) urlPath += 'index.html';
  // /sysid/*.js 從 sysid/ serve → 瀏覽器與 Node 共用同一份 kin.js/disturb_modes.js（SoT，不造第二真相源）
  const fromSysid = urlPath.startsWith('/sysid/') && urlPath.endsWith('.js');
  // 各自的服務沙箱：sysid 只到 sysid/，其餘只到 web/。
  const rootDir = fromSysid ? path.join(__dirname, 'sysid') : path.join(__dirname, 'web');
  const relPath = fromSysid ? urlPath.slice('/sysid'.length) : urlPath;
  const filePath = path.join(rootDir, relPath);
  // 防目錄遍歷：正規化後必須仍在自己的沙箱內（擋 /sysid/../server.js 之類 ../ 逃逸）
  if (filePath !== rootDir && !filePath.startsWith(rootDir + path.sep)) {
    res.writeHead(403); res.end('Forbidden'); return;
  }
  const ext = path.extname(filePath);
  fs.readFile(filePath, (err, data) => {
    if (err) { res.writeHead(404); res.end('Not found'); return; }
    res.writeHead(200, { 'Content-Type': MIME[ext] || 'text/plain' });
    res.end(data);
  });
});

// ===== WebSocket =====
const wss = new WebSocketServer({ server });
let clientCount = 0;

// 對等廣播匯流排：所有 client（monitor 頁、腳本）共用同一條送出路徑。
// telemetry（序列埠回覆）與指令 echo 都走這裡 → 任何來源的操作對所有人可見。
function broadcast(line) {
  for (const client of wss.clients) if (client.readyState === 1) client.send(line);
}

wss.on('connection', (ws, req) => {
  // 安全：拒絕跨站瀏覽器連線（惡意網頁經 DNS rebinding / CSWSH 可直送指令驅動實體馬達）。
  // 放行本機 + RFC1918 私網來源（保住站機台旁用手機/平板從 LAN IP 控制）；
  // 跨站攻擊頁的 Origin 是攻擊者公網域名、不符此段 → 擋掉。無 Origin 的 Node 腳本放行。
  const origin = req.headers.origin;
  const ALLOW_ORIGIN = /^https?:\/\/(localhost|127\.0\.0\.1|10\.\d+\.\d+\.\d+|192\.168\.\d+\.\d+|172\.(1[6-9]|2\d|3[01])\.\d+\.\d+)(:\d+)?$/;
  if (origin && !ALLOW_ORIGIN.test(origin)) {
    console.warn(`[WS] rejected cross-origin connection: ${origin}`);
    ws.close(1008, 'origin not allowed');
    return;
  }
  clientCount++;
  console.log(`[WS] client connected (${clientCount})`);

  // 立即送出最新資料
  if (lastData) ws.send(lastData);

  ws.on('message', (msg) => {
    const cmd = msg.toString().trim();
    if (!cmd) return;
    console.log(`[WS] cmd: ${cmd}`);
    recWrite('cmd', cmd);
    if (serial && serial.isOpen) {
      serial.write(cmd + '\n');
    }
    // echo 給所有 client（含其他來源）→ 操作對等可見，monitor 用此唯一來源畫指令標記
    broadcast(JSON.stringify({ evt: 'cmd', c: cmd }));
  });

  ws.on('close', () => {
    clientCount--;
    console.log(`[WS] client disconnected (${clientCount})`);
  });
});

// ===== Serial =====
let serial = null;
let uploadMode = false;

async function connectSerial() {
  if (uploadMode) return;
  const ports = await SerialPort.list();
  const usbPort = ports.find(p => p.path.includes('usbserial'));
  if (!usbPort) {
    console.error('[Serial] No USB serial port found. Retrying in 3s...');
    setTimeout(connectSerial, 3000);
    return;
  }

  // macOS: 用 cu. 開啟避免 tty. 的 carrier detect block
  const portPath = usbPort.path.replace('/dev/tty.', '/dev/cu.');
  console.log(`[Serial] opening ${portPath}`);
  serial = new SerialPort({ path: portPath, baudRate: BAUD });
  const parser = serial.pipe(new ReadlineParser({ delimiter: '\n' }));

  parser.on('data', (line) => {
    line = line.trim();
    if (!line) return;
    lastData = line;
    recWrite('in', line);
    broadcast(line);   // telemetry → 所有 client
  });

  serial.on('close', () => {
    console.log('[Serial] disconnected. Reconnecting in 3s...');
    serial = null;
    lastData = null;
    setTimeout(connectSerial, 3000);
  });

  serial.on('error', (err) => {
    console.error('[Serial] error:', err.message);
    serial = null;
    lastData = null;
    setTimeout(connectSerial, 3000);
  });
}

// ===== 啟動 =====
server.listen(HTTP_PORT, BIND_HOST, () => {
  console.log(`[Server] http://localhost:${HTTP_PORT}  (bind ${BIND_HOST})`);
  console.log(`[Server] REST: http://localhost:${HTTP_PORT}/api/latest`);
  if (BIND_HOST === '0.0.0.0')
    console.log('[Server] ⚠ 全網段可達。不可信網路請以 LOOPBACK_ONLY=1 啟動只綁本機。');
  connectSerial();
});
