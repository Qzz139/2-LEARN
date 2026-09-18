#!/usr/bin/env bash
# 真机入口：切换到仓库根目录，执行 SDK 脚本；默认行为由 Python 入口决定。
set -eo pipefail
cd "$(dirname "$0")/.."
exec python3 scripts/pick_real.py "$@"
