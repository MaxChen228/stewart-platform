#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

project_dir="$PWD"
mode="${1:-normal}"
port="${HTTP_PORT:-3000}"
https_port="${HTTPS_PORT:-3443}"

primary_ip() {
  local ip iface
  ip="$(ipconfig getifaddr en0 2>/dev/null || true)"
  if [[ -n "$ip" ]]; then
    printf '%s\n' "$ip"
    return
  fi
  ip="$(ipconfig getifaddr en1 2>/dev/null || true)"
  if [[ -n "$ip" ]]; then
    printf '%s\n' "$ip"
    return
  fi
  iface="$(route -n get default 2>/dev/null | awk '/interface:/{print $2; exit}' || true)"
  if [[ -n "$iface" ]]; then
    ip="$(ipconfig getifaddr "$iface" 2>/dev/null || true)"
    if [[ -n "$ip" ]]; then
      printf '%s\n' "$ip"
      return
    fi
  fi
  ifconfig | awk '/inet / && $2 != "127.0.0.1" {print $2; exit}'
}

ports=("$port")
if [[ "$mode" == "phone" ]]; then
  ports+=("$https_port")
elif [[ "$mode" != "normal" && "$mode" != "start" ]]; then
  echo "Usage: ./start.sh [phone]"
  exit 2
fi

existing_pids=""
for p in "${ports[@]}"; do
  while IFS= read -r pid; do
    [[ -z "$pid" ]] && continue
    cwd="$(lsof -a -p "$pid" -d cwd -Fn 2>/dev/null | sed -n 's/^n//p' || true)"
    if [[ "$cwd" == "$project_dir" && " $existing_pids " != *" $pid "* ]]; then
      existing_pids+="${existing_pids:+ }$pid"
    fi
  done < <(lsof -tiTCP:"$p" -sTCP:LISTEN 2>/dev/null || true)
done

if [[ -n "$existing_pids" ]]; then
  echo "Stopping existing Stewart server..."
  kill $existing_pids
  for _ in {1..30}; do
    still_running=""
    for pid in $existing_pids; do
      if kill -0 "$pid" 2>/dev/null; then
        still_running=1
        break
      fi
    done
    if [[ -z "$still_running" ]]; then
      break
    fi
    sleep 0.1
  done
fi

ip="$(primary_ip || true)"
if [[ -z "$ip" ]]; then
  ip="localhost"
fi

if [[ "$mode" == "phone" ]]; then
  echo
  echo "Phone IMU:"
  echo "  https://${ip}:${https_port}/phone.html"
  echo
  echo "Desktop:"
  echo "  http://${ip}:${port}/"
  echo
  echo "If the phone warns about the certificate, accept/trust the local dev cert."
  echo
  exec npm run start:phone
fi

echo
echo "Dashboard:"
echo "  http://${ip}:${port}/"
echo
exec npm start
