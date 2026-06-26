#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

project_dir="$PWD"
port="${HTTP_PORT:-3000}"

existing_pids=""
while IFS= read -r pid; do
  [[ -z "$pid" ]] && continue
  cwd="$(lsof -a -p "$pid" -d cwd -Fn 2>/dev/null | sed -n 's/^n//p' || true)"
  if [[ "$cwd" == "$project_dir" ]]; then
    existing_pids+="${existing_pids:+ }$pid"
  fi
done < <(lsof -tiTCP:"$port" -sTCP:LISTEN 2>/dev/null || true)

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

exec npm start
