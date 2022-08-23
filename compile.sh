#!/bin/bash

# Ugh. Doesn't provide correct content-type for js
echo sudo systemctl start nginx

while inotifywait -e close_write src/ --exclude \\..*; do
  echo rebuilding
  # --release
  wasm-pack build -d html/js --dev --no-typescript --target=web -- --color=always 2>&1 | less -RFX +gq #head -n $(tput lines)
done
