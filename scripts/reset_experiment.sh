#!/bin/bash
# Reset experiment ll_v1 to its initial checkpoint and clear runtime state.
set -euo pipefail

HERE=$(cd "$(dirname "$0")/.." && pwd)
cd "$HERE"

if [ -x scripts/server_ctl.sh ]; then
  scripts/server_ctl.sh stop || true
fi

rm -rf experiments/ll_v1/skills experiments/ll_v1/logs \
       experiments/ll_v1/library experiments/ll_v1/out
rm -f experiments/ll_v1/checkpoints/round_*.db \
      experiments/ll_v1/checkpoints/round_*.db-*
rm -f experiments/ll_v1/skill_system.db experiments/ll_v1/skill_system.db-*

python3 graph_cli.py --db experiments/ll_v1/skill_system.db reset

rm -rf queue/* .scratch/* run/*
echo "reset ll_v1 to the initial checkpoint"
