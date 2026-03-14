const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');
const { WebSocketServer } = require('ws');
const http = require('http');
const fs = require('fs');
const path = require('path');

const HTTP_PORT = 3000;
const BAUD = 115200;

// 最新一筆資料（供 REST API 用）
let lastData = null;

// ===== HTTP 靜態檔案伺服器 =====
const MIME = { '.html':'text/html', '.js':'application/javascript', '.css':'text/css', '.json':'application/json' };

const server = http.createServer((req, res) => {
  // REST: 最新資料
  if (req.url === '/api/latest') {
    res.writeHead(200, { 'Content-Type': 'application/json' });
    res.end(lastData || '{}');
    return;
  }

  // 靜態檔案
  const filePath = path.join(__dirname, 'web', req.url === '/' ? 'index.html' : req.url);
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

wss.on('connection', (ws) => {
  clientCount++;
  console.log(`[WS] client connected (${clientCount})`);

  // 立即送出最新資料
  if (lastData) ws.send(lastData);

  ws.on('message', (msg) => {
    const cmd = msg.toString().trim();
    console.log(`[WS] cmd: ${cmd}`);
    if (serial && serial.isOpen) {
      serial.write(cmd + '\n');
    }
  });

  ws.on('close', () => {
    clientCount--;
    console.log(`[WS] client disconnected (${clientCount})`);
  });
});

// ===== Serial =====
let serial = null;

async function connectSerial() {
  const ports = await SerialPort.list();
  const usbPort = ports.find(p => p.path.includes('cu.usbserial'));
  if (!usbPort) {
    console.error('[Serial] No USB serial port found. Retrying in 3s...');
    setTimeout(connectSerial, 3000);
    return;
  }

  console.log(`[Serial] opening ${usbPort.path}`);
  serial = new SerialPort({ path: usbPort.path, baudRate: BAUD });
  const parser = serial.pipe(new ReadlineParser({ delimiter: '\n' }));

  parser.on('data', (line) => {
    line = line.trim();
    if (!line) return;
    lastData = line;

    // 廣播給所有 WebSocket client
    for (const client of wss.clients) {
      if (client.readyState === 1) client.send(line);
    }
  });

  serial.on('close', () => {
    console.log('[Serial] disconnected. Reconnecting in 3s...');
    serial = null;
    setTimeout(connectSerial, 3000);
  });

  serial.on('error', (err) => {
    console.error('[Serial] error:', err.message);
  });
}

// ===== 啟動 =====
server.listen(HTTP_PORT, () => {
  console.log(`[Server] http://localhost:${HTTP_PORT}`);
  console.log(`[Server] REST: http://localhost:${HTTP_PORT}/api/latest`);
  connectSerial();
});
