"""32k-token compaction for the actor loop: summarize, don't truncate."""
from .llm import chat

BUDGET = 32000          # tokens; ~4 chars/token


def est(messages):
    return sum(len(str(m)) for m in messages) // 4


def compact(messages, budget=BUDGET, keep_recent=6, model=None):
    if est(messages) <= budget:
        return messages
    head, tail = messages[:1], messages[1:]
    # keep the newest exchanges verbatim, summarize everything older
    old, recent = tail[:-keep_recent], tail[-keep_recent:]
    txt = str(old)[:80000]
    summary, _ = chat([{'role': 'user', 'content':
                        'Summarize this agent history for continuity. Keep: '
                        'the task, what was tried and why, exact primitive '
                        'names/parameters that worked, exact failures with '
                        'numbers, and the current hypothesis. Be terse.\n\n'
                        + txt}], max_tokens=1200, model=model)
    return head + [{'role': 'user', 'content':
                    '[COMPACTED HISTORY]\n' + summary}] + recent
