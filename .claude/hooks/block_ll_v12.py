#!/usr/bin/env python3
"""PreToolUse guard: block access to ll_v1/ll_v2 artifacts from the ll_v3 run.

Reads the hook payload on stdin and exits 2 (block) when the tool input names
an ll_v1/ll_v2 path or the archive of those runs. Exit 0 lets the call through.
"""
import json
import re
import sys

BLOCK = re.compile(
    r"ll_v[12]|hmg_archive|work/tracks|(^|/)library/"
    r"|PRIMITIVES\.md|coordinator_prompt\.txt|coordinator_skill\.md"
    r"|HMG_harness_overview", re.I)
FIELDS = ("file_path", "path", "pattern", "glob", "command", "notebook_path")


def main():
    try:
        payload = json.load(sys.stdin)
    except Exception:
        return 0
    tool_input = payload.get("tool_input") or {}
    haystack = "\n".join(str(tool_input.get(k, "")) for k in FIELDS)
    if BLOCK.search(haystack):
        sys.stderr.write(
            "Blocked: out of scope for the ll_v3 run (ll_v1/ll_v2, work/tracks, "
            "library). Use only experiments/ll_v3 and work/ll_v3.\n")
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
