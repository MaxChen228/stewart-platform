#!/usr/bin/env node

const { execFileSync } = require('child_process');
const fs = require('fs');
const path = require('path');

const root = path.join(__dirname, '..');
const host = process.argv.includes('--host')
  ? process.argv[process.argv.indexOf('--host') + 1]
  : (fs.existsSync(path.join(root, '.esp32-ip'))
      ? fs.readFileSync(path.join(root, '.esp32-ip'), 'utf8').trim()
      : '192.168.50.107');
const http = process.env.HTTP_PORT || '3000';
const recover = process.argv.includes('--recover');
const probeTcp = process.argv.includes('--probe');

function sh(cmd, args, opts = {}) {
  try {
    return execFileSync(cmd, args, { encoding: 'utf8', timeout: opts.timeout || 5000 }).trim();
  } catch (e) {
    return (e.stdout || e.stderr || e.message || '').toString().trim();
  }
}

async function api(name, method = 'GET') {
  try {
    const r = await fetch(`http://localhost:${http}/api/${name}`, { method, signal: AbortSignal.timeout(2000) });
    return await r.json();
  } catch (e) {
    return { error: e.message };
  }
}

function tcpOpen(host, port) {
  const out = sh('nc', ['-vz', '-w', '2', host, String(port)], { timeout: 3000 });
  return {
    ok: out.includes('succeeded') || out.includes('open'),
    detail: out,
  };
}

(async () => {
  let [transport, latest] = await Promise.all([api('transport'), api('latest')]);
  const ping = sh('ping', ['-c', '3', '-W', '1000', host], { timeout: 5000 });
  const owners = sh('lsof', ['-nP', `-iTCP@${host}:3333`], { timeout: 5000 });
  const tcp3333 = probeTcp ? tcpOpen(host, 3333) : { skipped: true, reason: 'single-client port; pass --probe to touch it' };
  const ageMs = transport.lastLineAt ? Date.now() - transport.lastLineAt : null;

  const verdict = [];
  if (transport.error) verdict.push('dashboard API unreachable');
  if (transport.state !== 'connected') verdict.push(`transport ${transport.state || 'unknown'}:${transport.detail || ''}`);
  if (ageMs != null && ageMs > 3000) verdict.push(`telemetry stale ${ageMs}ms`);
  if (ping.includes('100.0% packet loss')) verdict.push('ESP32 does not answer ping');
  if (probeTcp && !tcp3333.ok) verdict.push('ESP32 TCP :3333 is not open');
  if (!owners.includes('node') && transport.state === 'connected') verdict.push('transport says connected but no TCP owner found');
  if (transport.state !== 'connected' && !ping.includes('100.0% packet loss')) {
    verdict.push('ESP32 reachable but dashboard transport is not connected');
    if (recover && !transport.error) {
      await api('transport/reconnect', 'POST');
      await new Promise((resolve) => setTimeout(resolve, 3500));
      [transport, latest] = await Promise.all([api('transport'), api('latest')]);
    }
  }

  console.log(JSON.stringify({
    host,
    transport,
    latest: latest && latest.t != null ? { t: latest.t, pos: latest.pos, lhz: latest.lhz, ef: latest.ef, tx: latest.tx, rx: latest.rx } : latest,
    telemetryAgeMs: ageMs,
    pingSummary: ping.split('\n').slice(-2).join('\n'),
    tcp3333,
    tcpOwners: owners || '(none)',
    verdict: verdict.length ? verdict : ['ok'],
  }, null, 2));
})();
