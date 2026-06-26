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
