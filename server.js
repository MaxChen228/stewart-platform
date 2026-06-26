const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');
const { WebSocketServer } = require('ws');
const http = require('http');
const net = require('net');
const fs = require('fs');
const path = require('path');

const HTTP_PORT = 3000;
const BAUD = 115200;

// ===== 傳輸選擇（env，預設 serial = 既有行為不變）=====
const TRANSPORT  = process.env.TRANSPORT || 'serial';        // serial | tcp
const ESP32_HOST = process.env.ESP32_HOST || '';             // tcp 模式必填
const ESP32_PORT = Number(process.env.ESP32_PORT) || 3333;
const TCP_IDLE_MS = Number(process.env.TCP_IDLE_MS) || 2000; // read-idle 半開偵測
// 燒錄抑制重連時長：tcp 較長（吸收 WiFi association 延遲）
const RELEASE_HOLD_MS = Number(process.env.RELEASE_HOLD_MS) || (TRANSPORT === 'tcp' ? 45000 : 30000);

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

  // 釋放傳輸（供韌體上傳用，暫停重連）。語意 transport-aware（見 transport.release）。
  if (req.url === '/api/release') {
    transport.release(RELEASE_HOLD_MS);
    res.writeHead(200, { 'Content-Type': 'application/json' });
    res.end('{"released":true}');
    return;
  }

  // 靜態檔案（去查詢字串；目錄/結尾 → index.html）
  let urlPath = req.url.split('?')[0];
  if (urlPath === '/') urlPath = '/index.html';
  else if (urlPath.endsWith('/')) urlPath += 'index.html';
  // /sysid/*.js 從專案根 serve → 瀏覽器與 Node 共用同一份 kin.js/disturb_modes.js（SoT，不造第二真相源）
  const fromSysid = urlPath.startsWith('/sysid/') && urlPath.endsWith('.js') && !urlPath.includes('..');
  const filePath = fromSysid ? path.join(__dirname, urlPath) : path.join(__dirname, 'web', urlPath);
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

wss.on('connection', (ws) => {
  clientCount++;
  console.log(`[WS] client connected (${clientCount})`);

  // 立即送出最新資料
  if (lastData) ws.send(lastData);

  ws.on('message', (msg) => {
    const cmd = msg.toString().trim();
    if (!cmd) return;
    console.log(`[WS] cmd: ${cmd}`);
    recWrite('cmd', cmd);
    transport.write(cmd);     // '\n' 由 transport 內部補；未連線則靜默 drop
    // echo 給所有 client（含其他來源）→ 操作對等可見，monitor 用此唯一來源畫指令標記
    broadcast(JSON.stringify({ evt: 'cmd', c: cmd }));
  });

  ws.on('close', () => {
    clientCount--;
    console.log(`[WS] client disconnected (${clientCount})`);
  });
});

// ===== 傳輸層 =====
// 傳輸無關的「收到一行」處理：telemetry/狀態/ack 都走這 → broadcast 給所有 WS client。
// Serial 與 TCP（P6）共用，確保前端對底層傳輸零感知。
function onLine(line) {
  line = line.trim();
  if (!line) return;
  lastData = line;
  recWrite('in', line);
  broadcast(line);
}

// Serial transport：實作傳輸介面 { isReady, write, connect, release }。
// suppressReconnect 取代舊 uploadMode（降為 transport 內部狀態，由 release 控制）。
function createSerialTransport({ onLine, baud }) {
  let serial = null;
  let suppressReconnect = false;
  let pendingTimer = null;   // 單一待連線 timer：reconnect/release/retry 共用、互斥不堆疊

  // 排程下一次 connect；先清前一個 → 雙重 release（雙擊/upload retry）不會堆出多個 socket
  function schedule(ms, fn) {
    if (pendingTimer) clearTimeout(pendingTimer);
    pendingTimer = setTimeout(() => { pendingTimer = null; (fn || connect)(); }, ms);
  }

  async function connect() {
    if (suppressReconnect || serial) return;   // 既有 socket 即返回，防覆蓋洩漏
    const ports = await SerialPort.list();
    const usbPort = ports.find(p => p.path.includes('usbserial'));
    if (!usbPort) {
      console.error('[Serial] No USB serial port found. Retrying in 3s...');
      schedule(3000);
      return;
    }
    // macOS: 用 cu. 開啟避免 tty. 的 carrier detect block
    const portPath = usbPort.path.replace('/dev/tty.', '/dev/cu.');
    console.log(`[Serial] opening ${portPath}`);
    serial = new SerialPort({ path: portPath, baudRate: baud });
    serial.pipe(new ReadlineParser({ delimiter: '\n' })).on('data', onLine);

    const onGone = (why) => {
      console.log(`[Serial] ${why}. Reconnecting in 3s...`);
      serial = null;
      lastData = null;
      if (!suppressReconnect) schedule(3000);
    };
    serial.on('close', () => onGone('disconnected'));
    serial.on('error', (err) => { console.error(`[Serial] error: ${err.message}`); onGone('error'); });  // error 留 stderr
  }

  return {
    isReady() { return !!(serial && serial.isOpen); },
    write(line) { if (serial && serial.isOpen) serial.write(line + '\n'); },
    connect,
    release(holdMs) {
      suppressReconnect = true;
      if (serial && serial.isOpen) {
        serial.close();
        console.log(`[Serial] released for upload (${holdMs}ms hold)`);
      }
      schedule(holdMs, () => { suppressReconnect = false; connect(); });
    },
  };
}

// TCP transport（P6）：連 ESP32 WiFi 的 :3333，協議與 serial 逐字相同。
// 半開偵測用「手動 read-idle」（遙測連續流即 liveness）——不可用 socket.setTimeout，
// 因心跳 write 會重置它而漏判死 peer。心跳 write 獨立，讓韌體可選的 HB 失效保護生效。
function createTcpTransport({ onLine, host, port }) {
  let socket = null;
  let suppressReconnect = false;
  let hbTimer = null;       // 對 ESP32 週期送 '\n' 心跳（韌體 lastNetRxMs 基準；空行被忽略）
  let idleTimer = null;     // read-idle 檢查
  let pendingTimer = null;  // 單一待連線 timer：reconnect/release 共用、互斥不堆疊
  let lastRead = 0;

  function clearTimers() {
    if (hbTimer) { clearInterval(hbTimer); hbTimer = null; }
    if (idleTimer) { clearInterval(idleTimer); idleTimer = null; }
  }

  // 排程下一次 connect；先清前一個 → 雙重 release 不會堆出多個 socket/心跳 interval
  function schedule(ms, fn) {
    if (pendingTimer) clearTimeout(pendingTimer);
    pendingTimer = setTimeout(() => { pendingTimer = null; (fn || connect)(); }, ms);
  }

  function connect() {
    if (suppressReconnect || socket) return;   // 既有 socket 即返回，防覆蓋洩漏
    console.log(`[TCP] connecting ${host}:${port}`);
    socket = net.connect({ host, port });
    socket.setNoDelay(true);
    socket.setKeepAlive(true, 5000);
    socket.pipe(new ReadlineParser({ delimiter: '\n' })).on('data', (line) => {
      lastRead = Date.now();
      onLine(line);
    });
    socket.on('connect', () => {
      console.log('[TCP] connected');
      lastRead = Date.now();
      hbTimer = setInterval(() => { if (socket && !socket.destroyed && socket.writable) socket.write('\n'); }, 1000);
      idleTimer = setInterval(() => {
        if (socket && !socket.destroyed && Date.now() - lastRead > TCP_IDLE_MS) {
          console.log('[TCP] read-idle, destroying');
          socket.destroy();   // → close → 重連；ESP32 收 RST → 韌體 socket-close failsafe
        }
      }, 500);
    });
    const onGone = (why) => {
      console.log(`[TCP] ${why}. Reconnecting in 3s...`);
      clearTimers();
      socket = null;
      lastData = null;
      if (!suppressReconnect) schedule(3000);
    };
    socket.on('close', () => onGone('closed'));
    socket.on('error', (err) => console.error('[TCP] error:', err.message));  // close 隨後觸發 onGone
  }

  return {
    isReady() { return !!(socket && !socket.destroyed && socket.writable); },
    write(line) { if (socket && !socket.destroyed && socket.writable) socket.write(line + '\n'); },
    connect,
    release(holdMs) {
      suppressReconnect = true;
      clearTimers();
      if (socket && !socket.destroyed) {
        socket.destroy();
        console.log(`[TCP] released for upload (${holdMs}ms hold)`);
      }
      schedule(holdMs, () => { suppressReconnect = false; connect(); });
    },
  };
}

if (TRANSPORT === 'tcp' && !ESP32_HOST) {
  console.error('[Config] TRANSPORT=tcp 需設 ESP32_HOST（如 ESP32_HOST=192.168.1.50）');
  process.exit(1);
}
const transport = (TRANSPORT === 'tcp')
  ? createTcpTransport({ onLine, host: ESP32_HOST, port: ESP32_PORT })
  : createSerialTransport({ onLine, baud: BAUD });

// ===== 啟動 =====
server.listen(HTTP_PORT, () => {
  console.log(`[Server] http://localhost:${HTTP_PORT}`);
  console.log(`[Server] REST: http://localhost:${HTTP_PORT}/api/latest`);
  console.log(`[Server] transport: ${TRANSPORT}${TRANSPORT === 'tcp' ? ` → ${ESP32_HOST}:${ESP32_PORT}` : ''}`);
  transport.connect();
});
