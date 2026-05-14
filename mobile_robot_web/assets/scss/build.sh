#!/usr/bin/env bash
# Compile dashboard.scss into ../css/dashboard.css. Requires dart-sass.
# Install with:  sudo apt install sass   (Ubuntu 24.04+)
#            or  npm i -g sass
set -euo pipefail
cd "$(dirname "$0")"

if ! command -v sass >/dev/null 2>&1; then
  echo "sass not found. Install with: sudo apt install sass  OR  npm i -g sass" >&2
  exit 1
fi

sass --no-source-map dashboard.scss ../css/dashboard.css
echo "Compiled: ../css/dashboard.css"
