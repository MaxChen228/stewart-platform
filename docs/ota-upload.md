# WiFi OTA Upload

This project supports ArduinoOTA over the existing WiFi station connection.

## One-time setup

1. Flash OTA-enabled firmware once over USB:

   ```bash
   npm run upload
   ```

2. Make sure WiFi credentials are saved and WiFi is enabled:

   ```text
   WIFI <ssid> <password>
   WIFION
   WIFI?
   ```

`WIFI?` reports `wifi.ota.ready=1` when OTA is available. The default hostname is
`stewart.local`.

## Upload over WiFi

Before uploading, make sure only one process owns the ESP32 TCP transport:

```bash
lsof -nP -iTCP@192.168.x.x:3333
```

`npm run upload:ota` asks the dashboard server to release its TCP connection
before OTA. This matters because ESP32 `:3333` is single-client: another
dashboard/server from a different worktree can repeatedly steal the socket and
make WiFi look like it is dropping every ~1.5s.

Use mDNS:

```bash
npm run upload:ota
```

Or target the IP directly:

```bash
npm run upload:ota -- --host 192.168.x.x
```

You can also put the IP in `.esp32-ip`; the script will use that before falling
back to `stewart.local`.

## Safety behavior

When OTA starts, firmware immediately leaves position control, stops position
commands, and disables all motors before flash writing begins. The board will
reboot after a successful upload.

Do not start OTA while the platform is carrying a load that cannot safely lose
motor torque. Keep USB upload available as the recovery path if WiFi or OTA is
not reachable.
