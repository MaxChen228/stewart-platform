"""PlatformIO pre-build hook: 把韌體源碼身分烤進 src/fw_identity.h。
單一演算法在 sysid/fw_hash.js（node），此 shim 只負責在編譯前呼它。
node 不可用時退化寫 placeholder header → build 仍可過（身分降級為 nohash）。"""
Import("env")  # noqa: F821  (PlatformIO SCons 注入)
import os
import subprocess

ROOT = env.subst("$PROJECT_DIR")
SCRIPT = os.path.join(ROOT, "sysid", "fw_hash.js")
HEADER = os.path.join(ROOT, "src", "fw_identity.h")

try:
    subprocess.run(["node", SCRIPT, "--emit-header", HEADER], check=True)
except Exception as e:  # noqa: BLE001  (node 缺失/失敗皆不可阻斷 build)
    print(f"[fw_stamp] node hash 失敗 ({e})；寫入 placeholder 身分")
    with open(HEADER, "w") as f:
        f.write(
            '// fw_stamp fallback (node unavailable)\n'
            '#define FW_SRC_HASH "nohash"\n'
            '#define FW_GIT_REV "nohash"\n'
            '#define FW_DIRTY 0\n'
            '#define FW_BUILD_EPOCH 0UL\n'
        )
