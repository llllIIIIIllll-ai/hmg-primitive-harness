#!/bin/bash
# usage: [HMG_TASK=<task>] qtool.sh '<json request>' [timeout_seconds]
# Writes a fresh req_<id>.json into this track's queue and waits for its
# response.  HMG_TASK selects queue/<task>/ (one queue per track); without it
# the request goes to queue/shared/, which no track server polls, so a
# forgotten HMG_TASK shows up as a timeout.
HERE=$(cd "$(dirname "$0")" && pwd)
QDIR="$HERE/queue/${HMG_TASK:-shared}"
REQ="$1"
TMO="${2:-120}"
mkdir -p "$QDIR"
ID="a$(date +%s%N | tail -c 12)"
echo "$REQ" > "$QDIR/req_$ID.json"
for i in $(seq 1 $((TMO*10))); do
  if [ -s "$QDIR/resp_$ID.json" ]; then
    cat "$QDIR/resp_$ID.json"
    echo
    exit 0
  fi
  sleep 0.1
done
echo "TIMEOUT waiting for $QDIR/resp_$ID.json"
exit 1
