#!/usr/bin/env node

const { spawnSync } = require('child_process');
const fs = require('fs');
const os = require('os');
const path = require('path');

function usage() {
  console.log([
    'Usage: npm run upload:ota -- [--host <ip-or-hostname>]',
    '',
    'Defaults to .esp32-ip when present, otherwise stewart.local.',
    'The ESP32 must already be running OTA-enabled firmware on the same WiFi.',
  ].join('\n'));
}

let host = null;
for (let i = 2; i < process.argv.length; i++) {
  const arg = process.argv[i];
  if (arg === '--help' || arg === '-h') {
    usage();
    process.exit(0);
  }
  if (arg === '--host') {
    host = process.argv[++i];
    continue;
  }
  if (arg.startsWith('--host=')) {
    host = arg.slice('--host='.length);
    continue;
  }
  console.error(`Unknown argument: ${arg}`);
  usage();
  process.exit(2);
}

if (!host) {
  const ipFile = path.join(process.cwd(), '.esp32-ip');
  if (fs.existsSync(ipFile)) host = fs.readFileSync(ipFile, 'utf8').trim();
}
if (!host) host = 'stewart.local';

const pio = path.join(os.homedir(), '.platformio', 'penv', 'bin', 'pio');
const args = ['run', '-e', 'esp32_ota', '-t', 'upload', '--upload-port', host];
const releaseMs = Number(process.env.OTA_RELEASE_MS) || 90000;

console.log(`OTA upload target: ${host}`);
console.log('Safety: firmware will disable motors when OTA starts.');

const curl = spawnSync('curl', ['-fsS', `http://localhost:3000/api/release?ms=${releaseMs}`], {
  stdio: 'pipe',
  encoding: 'utf8',
  timeout: 3000,
});
if (curl.status === 0) {
  console.log(`Released dashboard transport for ${releaseMs}ms.`);
  spawnSync('sleep', ['1']);
} else {
  console.warn('Warning: could not release dashboard transport; continuing OTA anyway.');
  if (curl.stderr) console.warn(curl.stderr.trim());
}

const result = spawnSync(pio, args, { stdio: 'inherit' });
process.exit(result.status ?? 1);
