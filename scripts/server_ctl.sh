#!/bin/bash
# Local (non-Slurm) server control for the HMG harness.
# Usage: ./server_ctl.sh start <task> [seed] [slot] | stop [task] | status
# A slot starts an additional server for the same task with its own queue
# (queue/<task>__<slot>), so several execution actors can drive one task in
# parallel.  Slot 0 / no slot is the original per-task queue.
set -euo pipefail

HERE=$(cd "$(dirname "$0")/.." && pwd)
VENV=${HMG_VENV:-$HERE/.venv}
REPO=${HMG_REPO:-$HERE/third_party/humanoidmimicgen}
export HMG_REPO
export MUJOCO_GL=${MUJOCO_GL:-egl}
RUN=$HERE/run
LOGDIR=$HERE/.scratch
mkdir -p "$RUN" "$LOGDIR"

up() { [ -f "$RUN/$1.pid" ] && kill -0 "$(cat "$RUN/$1.pid")" 2>/dev/null; }

start() {
  local task=$1 seed=${2:-0} slot=${3:-0}
  local name=$task
  [ "$slot" != "0" ] && name="${task}__${slot}"
  if up "$name"; then
    echo "already running $name pid $(cat "$RUN/$name.pid")"
    return
  fi
  export HMG_QUEUE="$HERE/queue/$name"
  mkdir -p "$HMG_QUEUE"
  cd "$HERE"
  local log="$LOGDIR/srv_${name}_$(date +%H%M%S).log"
  nohup nice -n 10 "$VENV/bin/python" "$HERE/hmg_server.py" "$task" "$seed" > "$log" 2>&1 &
  echo $! > "$RUN/$name.pid"
  echo "started $name seed=$seed pid=$(cat "$RUN/$name.pid") queue=$HMG_QUEUE log=$log"
}

status() {
  local f task pid
  for f in "$RUN"/*.pid; do
    [ -e "$f" ] || continue
    task=$(basename "$f" .pid)
    pid=$(cat "$f")
    if kill -0 "$pid" 2>/dev/null; then
      echo "running $task pid $pid"
    else
      echo "dead $task (stale pid $pid)"
    fi
  done
}

stop() {
  local task=${1:-} f
  if [ -z "$task" ]; then
    for f in "$RUN"/*.pid; do
      [ -e "$f" ] || continue
      kill "$(cat "$f")" 2>/dev/null || true
      rm -f "$f"
    done
    echo "stopped all"
    return
  fi
  for f in "$RUN/$task.pid" "$RUN/$task"__*.pid; do
    [ -e "$f" ] || continue
    kill "$(cat "$f")" 2>/dev/null || true
    rm -f "$f"
  done
  echo "stopped $task (all slots)"
}

case "${1:-}" in
  start) start "${2:?task}" "${3:-0}" "${4:-0}" ;;
  stop) stop "${2:-}" ;;
  status) status ;;
  *) echo "usage: $0 {start <task> [seed]|stop [task]|status}"; exit 1 ;;
esac
