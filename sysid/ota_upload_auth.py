"""PlatformIO post-build hook（僅 [env:esp32_ota]）：為 espota 上傳注入 --auth 與 -I host_ip。

為何 post:（非 pre:）——平台 builder/main.py 對 espota 做 env.Replace(UPLOADERFLAGS=[...])，
會清掉 pre-script 早期的 append；post: 在其之後執行才留得住。

- --auth <密碼>：從 gitignored secrets.json / ENV OTA_PASSWORD 取（比照 ota_pass.py，密碼不入 git）。
  無值則不帶（對應開放韌體）。
- -I <host_ip>：espota 邀請封包要告訴板子「連回哪個 host IP」。多網卡（如 Tailscale）時 espota
  自動偵測會得 0.0.0.0 → 板子連不回 → "Host Not Found"。這裡用「路由到板子的本地位址」精準取，
  免硬編 IP、免受 Tailscale 干擾。upload_port 為主機名則先解析。取不到就跳過（退回 espota 預設）。
"""
Import("env")  # noqa: F821
import os
import json
import socket

if env.subst("$PIOENV") == "esp32_ota":
    ROOT = env.subst("$PROJECT_DIR")
    SECRETS = os.path.join(ROOT, "sysid", "config", "secrets.json")

    def _pw():
        pw = os.environ.get("OTA_PASSWORD")
        if pw:
            return pw.strip()
        try:
            with open(SECRETS) as f:
                v = json.load(f).get("otaPassword")
            return v.strip() if isinstance(v, str) else ""
        except Exception:  # noqa: BLE001
            return ""

    pw = _pw()
    if pw:
        env.Append(UPLOADERFLAGS=["--auth", pw])

    # 路由導向的本地位址（解 Tailscale 多網卡誤判 host_ip）
    port = env.subst("$UPLOAD_PORT")
    try:
        board_ip = socket.gethostbyname(port)
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect((board_ip, 3232))          # UDP：不實際送封包，只取路由本地端
        host_ip = s.getsockname()[0]
        s.close()
        if host_ip and host_ip != "0.0.0.0":
            env.Append(UPLOADERFLAGS=["-I", host_ip])
            print(f"[ota_upload_auth] espota host_ip={host_ip} auth={'yes' if pw else 'no'}")
    except Exception as e:  # noqa: BLE001
        print(f"[ota_upload_auth] host_ip 自動偵測略過（{e}）；auth={'yes' if pw else 'no'}")
