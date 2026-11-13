"""Actor-authored primitives: the agent WRITES and DEBUGS new skills.

A primitive is Python source with one entry point:

    def run(api, **kwargs):
        api.observe(); api.set_height(...); api.reach_box(...)
        return {"status": ...}

It is compiled with an EMPTY __builtins__ and only ``np``/``math`` in scope, and
receives the trusted API object -- never the simulator, reward or predicate.

Composition (2026-11-10, fixer): a registered primitive may call another
registered primitive with ``api.skill(name, **kwargs)`` and gets its returned
dict back.  The call uses the same api object (one env, one episode), is
resolved from the registry at call time, and is depth-capped so a self-call or
a cycle fails with RecursionError instead of recursing until the process dies.
"""
import math
import os
import sys

import numpy as np

FORBIDDEN = ('__import__', 'open(', 'eval(', 'exec(', 'os.', 'sys.',
             'subprocess', 'socket', 'builtins')

# api.skill() nesting cap, harness-side only: read once from the server's
# environment when this module is imported (HMG_SKILL_MAX_DEPTH, default 8).
# Primitive source is exec'd with an empty globals dict (no os, no this
# module) and there is no api attribute or keyword that exposes the cap.
MAX_SKILL_DEPTH = max(1, int(os.environ.get('HMG_SKILL_MAX_DEPTH', '8')))


def _primitive_frames():
    """How many exec'd primitive ``run`` frames are on the current stack."""
    n = 0
    f = sys._getframe(1)
    while f is not None:
        if f.f_code.co_filename.startswith('<primitive:'):
            n += 1
        f = f.f_back
    return n


class PrimitiveRuntime:
    def __init__(self, api):
        self.reg = {}
        self.api = api

    @property
    def api(self):
        return self._api

    @api.setter
    def api(self, tools):
        # Bind the composition hook onto the api the primitives already
        # receive.  This is a property so the in-process episode restart
        # (``RT.api = tools`` in hmg_server.new_episode) rebinds it too.
        self._api = tools
        tools.skill = self._skill

    def register(self, name, source):
        bad = [w for w in FORBIDDEN if w in source]
        if bad:
            raise ValueError('forbidden in primitive source: %s' % bad)
        g = {'__builtins__': {}, 'np': np, 'math': math}
        exec(compile(source, '<primitive:%s>' % name, 'exec'), g)
        fn = g.get('run')
        if not callable(fn):
            raise ValueError('source must define run(api, **kwargs)')
        self.reg[name] = fn
        return sorted(self.reg)

    def call(self, name, **kw):
        if name not in self.reg:
            raise KeyError('no primitive %r; have %s'
                           % (name, sorted(self.reg)))
        t0 = self.api.t
        out = self.reg[name](self.api, **kw)
        if not isinstance(out, dict):
            out = {'returned': str(out)[:200]}
        out.setdefault('t0', round(t0, 2))
        out.setdefault('t1', round(self.api.t, 2))
        return out

    def _skill(self, name, **kw):
        """api.skill(name, **kwargs): run another registered primitive.

        Same api object, so nested calls compose through one env.  The inner
        dict is returned unchanged (call() supplies its t0/t1 defaults).
        """
        d = _primitive_frames()
        if d >= MAX_SKILL_DEPTH:
            raise RecursionError('api.skill depth limit %d exceeded calling %r'
                                 % (MAX_SKILL_DEPTH, name))
        return self.call(name, **kw)
