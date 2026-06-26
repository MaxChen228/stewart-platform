'use strict';

const http = require('http');
const https = require('https');
const WebSocket = require('ws');
const { NEUTRAL_Z } = require('./kin');
const PlatformSoT = require('./platform_sot');

function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

function parsePose(s) {
  const p = String(s).split(',').map((x) => Number(x.trim()));
  if (!PlatformSoT.finitePose(p)) throw new Error(`Bad pose: ${s}`);
  return p;
}

function poseLine(pose) {
  return pose.map((x) => Number(x).toFixed(3)).join(' ');
}

function safeName(s) {
  return String(s || 'run').replace(/[^a-zA-Z0-9_-]/g, '_');
}

function httpBase(host) {
  return host.startsWith('http://') || host.startsWith('https://') ? host : `http://${host}`;
}

function wsUrl(host) {
  const u = new URL(httpBase(host));
  u.protocol = u.protocol === 'https:' ? 'wss:' : 'ws:';
  u.pathname = '/';
  return u.toString();
}

function rest(host, apiPath) {
  const url = new URL(apiPath.startsWith('/api/') ? apiPath : `/api/${apiPath}`, httpBase(host));
  const mod = url.protocol === 'https:' ? https : http;
  return new Promise((resolve, reject) => {
    const req = mod.get(url, (res) => {
      let data = '';
      res.on('data', (c) => { data += c; });
      res.on('end', () => {
        if (res.statusCode < 200 || res.statusCode >= 300) {
          reject(new Error(`HTTP ${res.statusCode} for ${url.pathname}`));
          return;
        }
        try { resolve(JSON.parse(data || '{}')); }
        catch (e) { reject(e); }
      });
    });
    req.on('error', reject);
  });
}

async function openWs(host, timeoutMs = 3000) {
  const ws = new WebSocket(wsUrl(host));
  await new Promise((resolve, reject) => {
    const timer = setTimeout(() => reject(new Error('WebSocket timeout')), timeoutMs);
    ws.on('open', () => { clearTimeout(timer); resolve(); });
    ws.on('error', reject);
  });
  return ws;
}

function send(ws, cmd, opts = {}) {
  if (opts.log !== false) console.log(`> ${cmd}`);
  ws.send(cmd);
}

function relToAbs(rel) {
  return PlatformSoT.relToAbs(rel, NEUTRAL_Z);
}

function absToRel(pose) {
  return PlatformSoT.absToRel(pose, NEUTRAL_Z);
}

async function loadPlatformConfig(host) {
  return PlatformSoT.mergeConfig(await rest(host, '/api/platform-config'));
}

async function loadHomePose(host, fallbackRel = PlatformSoT.DEFAULT_PLATFORM_CONFIG.homeRelative) {
  try {
    const d = await rest(host, '/api/platform-config');
    if (PlatformSoT.finitePose(d.homePose)) return d.homePose;
    if (PlatformSoT.finitePose(d.homeRelative)) return relToAbs(d.homeRelative);
  } catch {}
  return relToAbs(fallbackRel);
}

async function loadLandingPose(host, fallbackRel = PlatformSoT.DEFAULT_PLATFORM_CONFIG.landingRelative, explicitRel = null) {
  if (explicitRel) return relToAbs(explicitRel);
  try {
    const d = await rest(host, '/api/platform-config');
    if (PlatformSoT.finitePose(d.landingPose)) return d.landingPose;
    if (PlatformSoT.finitePose(d.landingRelative)) return relToAbs(d.landingRelative);
  } catch {}
  return relToAbs(fallbackRel);
}

async function startRecording(host, name) {
  return rest(host, `/api/rec/start?name=${encodeURIComponent(name)}`);
}

async function stopRecording(host) {
  return rest(host, '/api/rec/stop');
}

module.exports = {
  WebSocket,
  PlatformSoT,
  NEUTRAL_Z,
  sleep,
  parsePose,
  poseLine,
  safeName,
  httpBase,
  wsUrl,
  rest,
  openWs,
  send,
  relToAbs,
  absToRel,
  loadPlatformConfig,
  loadHomePose,
  loadLandingPose,
  startRecording,
  stopRecording,
};
