#!/bin/bash

python3 -m http.server 8080 &
child_pid="$!"

cleanup() {
  kill "$child_pid"
}
trap cleanup INT KILL TERM

while inotifywait -e close_write src/ --exclude \\..*; do
  echo rebuilding
  # --release
  wasm-pack build -d html/js --dev --no-typescript --target=web -- --color=always 2>&1 | head -n $(tput lines)
done
cleanup
