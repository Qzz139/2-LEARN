#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
exec python3 scripts/pick_real.py "$@"
