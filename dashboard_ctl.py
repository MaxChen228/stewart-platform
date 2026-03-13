#!/usr/bin/env python3

from __future__ import annotations

import argparse
import os
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parent
PID_PATH = ROOT / ".dashboard.pid"
LOG_PATH = ROOT / ".dashboard.log"
PORT = 8080


def pid_is_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def read_pidfile() -> int | None:
    if not PID_PATH.exists():
        return None
    try:
        return int(PID_PATH.read_text().strip())
    except Exception:
        return None


def remove_pidfile() -> None:
    try:
        PID_PATH.unlink()
    except FileNotFoundError:
        pass


def port_open() -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(0.25)
        return sock.connect_ex(("127.0.0.1", PORT)) == 0


def pids_using_port() -> list[int]:
    result = subprocess.run(
        ["lsof", "-ti", f"tcp:{PORT}"],
        capture_output=True,
        text=True,
        check=False,
    )
    pids: list[int] = []
    for line in result.stdout.splitlines():
        line = line.strip()
        if line.isdigit():
            pids.append(int(line))
    return pids


def stop_server() -> list[int]:
    targets = set()
    pid = read_pidfile()
    if pid:
        targets.add(pid)
    targets.update(pids_using_port())

    stopped: list[int] = []
    for target in sorted(targets):
        if not pid_is_alive(target):
            continue
        try:
            os.kill(target, signal.SIGTERM)
            stopped.append(target)
        except ProcessLookupError:
            continue

    deadline = time.time() + 2.0
    while time.time() < deadline:
        alive = [pid for pid in stopped if pid_is_alive(pid)]
        if not alive:
            break
        time.sleep(0.05)

    for target in list(stopped):
        if pid_is_alive(target):
            try:
                os.kill(target, signal.SIGKILL)
            except ProcessLookupError:
                pass

    remove_pidfile()
    return stopped


def start_server() -> int:
    if port_open():
        existing = pids_using_port()
        if existing:
            return existing[0]

    log = LOG_PATH.open("ab")
    process = subprocess.Popen(
        [sys.executable, "dashboard.py"],
        cwd=ROOT,
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    PID_PATH.write_text(str(process.pid))

    deadline = time.time() + 5.0
    while time.time() < deadline:
        if port_open():
            return process.pid
        if process.poll() is not None:
            raise RuntimeError(f"dashboard exited early with code {process.returncode}")
        time.sleep(0.05)

    raise RuntimeError("dashboard did not open port 8080 in time")


def status() -> int:
    pid = read_pidfile()
    alive = pid is not None and pid_is_alive(pid)
    port = port_open()
    print(f"pidfile={pid if pid else '-'} alive={'yes' if alive else 'no'} port8080={'open' if port else 'closed'}")
    if LOG_PATH.exists():
        tail = LOG_PATH.read_text(errors="ignore").splitlines()[-5:]
        if tail:
            print("log tail:")
            for line in tail:
                print(line)
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Control the Stewart dashboard server")
    parser.add_argument("command", choices=["start", "stop", "restart", "status"])
    args = parser.parse_args()

    if args.command == "start":
        pid = start_server()
        print(f"started dashboard pid={pid}")
        return 0
    if args.command == "stop":
        stopped = stop_server()
        print(f"stopped={stopped or []}")
        return 0
    if args.command == "restart":
        stopped = stop_server()
        pid = start_server()
        print(f"stopped={stopped or []}")
        print(f"started dashboard pid={pid}")
        return 0
    return status()


if __name__ == "__main__":
    raise SystemExit(main())
