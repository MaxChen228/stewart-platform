"""PlatformIO pre-build hook：從 gitignored sysid/config/secrets.json 注入 OTA 密碼到 build。
- 有 otaPassword → 定義 -DOTA_PASSWORD="..."（兩個 env 皆烤入，燒錄後韌體 ArduinoOTA 要求此密碼）。
- 無檔/無值 → 不動（韌體 OTA_PASSWORD 預設 "" = 開放，向後相容）。
espota upload 端的 --auth 由 post-hook sysid/ota_upload_auth.py 注入（UPLOADERFLAGS 須在平台
main.py 的 env.Replace 之後 append，故用 post: 而非 pre:）。
密碼絕不入 git（比照 https-*.pem / platform.json 慣例）；金鑰真相 = 本機 secrets.json 或 ENV OTA_PASSWORD。"""
Import("env")  # noqa: F821  (PlatformIO SCons 注入)
import os
import json

ROOT = env.subst("$PROJECT_DIR")
SECRETS = os.path.join(ROOT, "sysid", "config", "secrets.json")


def _load_ota_password():
    # ENV 優先（CI/一次性覆寫），否則讀 gitignored secrets.json
    pw = os.environ.get("OTA_PASSWORD")
    if pw:
        return pw.strip()
    try:
        with open(SECRETS, "r") as f:
            data = json.load(f)
        v = data.get("otaPassword")
        return v.strip() if isinstance(v, str) else ""
    except Exception:  # noqa: BLE001  (缺檔/壞 json 皆退化為開放)
        return ""


pw = _load_ota_password()
if pw:
    # StringifyMacro 產出正確跳脫的 C 字串字面量 → net_transport.cpp #ifndef 分支被略過
    env.Append(CPPDEFINES=[("OTA_PASSWORD", env.StringifyMacro(pw))])
    print(f"[ota_pass] OTA 密碼已烤入 build（len={len(pw)}，env={env.subst('$PIOENV')}）")
else:
    print("[ota_pass] 無 OTA 密碼（secrets.json/ENV 未設）→ 韌體 OTA 開放，向後相容")
