"""Feedback loop: what the actor SEES and what it may ask for.

Always-on (one JSON per episode):
  success / safety (COORDINATOR-ONLY -- stripped from every actor-visible
  view), per-step phase log, the EARLIEST 1-3 decisive events with the exact
  time and the residual that fired them, per-link contact snapshots, and a
  small set of maxima (tilt, base drop, hand-object slip, joint-limit margin,
  action jump, measured torque).
On demand (trigger windows, 0.2 s before / after):
  sampled state / command / torque / object / residual arrays around one
  event, plus the contact dump captured at that step.

Design rule: an event is EVIDENCE, not a root cause -- the summary says what
fired first and where to look, never why.  The reward / success predicate is
never computed or stored here; ``save`` receives them only so the
coordinator's private copy keeps the historical fields, and ``public``
explicitly omits them.

Observability extension (2026-11-09, obs fixer): per-step arrays
(q, qd, torque, object pose, base height, tilt, contact counts), contact
snapshots (geom/body names, world normal, distance, penetration, contact
force), action-jump / torque-saturation / contact-penetration events, and a
window() that honours a time pad.  Every added argument is optional, so the
original Recorder contract (and ``hmg_harness/run.py``) keeps working.
"""
import json
import os

import numpy as np

EVENT_KINDS = (
    'JOINT_LIMIT', 'TORQUE_SATURATION', 'ACTION_JUMP', 'BASE_DROP',
    'TILT', 'SUPPORT_LOSS', 'HAND_CONTACT_LOSS', 'OBJECT_SLIP',
    'NO_PROGRESS', 'HANDOFF',    # controller-owned events, backend-reported
    'IK_UNREACHABLE', 'QP_INFEASIBLE',
    'CONTACT_PENETRATION',      # added: object/robot interpenetration
)

# Fields that may be requested from window()/save(); each maps to one of the
# per-step lists.  Kept explicit so a typo never silently returns nothing.
TRACE_FIELDS = ('t', 'q', 'qd', 'act', 'tau', 'obj', 'obj_quat', 'obj_lo',
                'obj_hi', 'base_z', 'tilt', 'hand_n', 'contact_n', 'nav',
                'height')


class Recorder:
    def __init__(self, out_dir, task, seed, program_hash, dt=0.02,
                 limits=None, thresholds=None, meta=None, contact_every=10,
                 max_contacts=80):
        self.dir = out_dir
        os.makedirs(out_dir, exist_ok=True)
        self.meta = {'task': task, 'seed': int(seed),
                     'program_hash': program_hash}
        if meta:
            self.meta.update(meta)
        self.dt = float(dt)
        self.limits = limits            # (nq,) lo, (nq,) hi, or None
        self.th = dict(lim_margin=0.02, act_jump=0.35, base_drop=0.08,
                       tilt=0.6, slip=0.03, hold_steps=25,
                       penetration=0.005, torque_frac=0.995)
        self.th.update(thresholds or {})
        self.contact_every = max(1, int(contact_every))
        self.max_contacts = max(1, int(max_contacts))
        # raw (immutable)
        self.t, self.q, self.qd, self.a = [], [], [], []
        self.extra = []
        # observability extension: per-step time series (missing -> None)
        self.tau = []
        self.obj = []
        self.obj_quat = []
        self.obj_lo = []
        self.obj_hi = []
        self.base_z = []
        self.tilt = []
        self.hand_n = []
        self.contact_n = []
        self.nav = []
        self.height = []
        # full per-link contact snapshots (every contact_every steps plus
        # every step at which an event fired)
        self.contact_snapshots = []
        # running diagnostics
        self.events = []
        self._last_fire = {}
        self.max = {'tilt': 0., 'base_drop': 0., 'slip': 0.,
                    'act_jump': 0., 'lim_margin': 1e9,
                    'torque': 0., 'torque_frac': 0.,
                    'penetration': 0., 'penetration_all': 0.,
                    'hand_contacts': 0, 'ncon': 0}
        self._base0 = None
        self._prog = None
        self._still = 0
        self._had_contact = False
        self._step = 0
        self._final = None

    # ---- per step ------------------------------------------------------
    def add(self, t, q, qd, a, base_z=None, tilt=0., obj_pos=None,
            hand_contacts=None, progress=None, torque=None,
            torque_limit=None, contacts=None, nav=None, height=None,
            obj_quat=None, obj_aabb=None, max_penetration=None,
            max_penetration_all=None):
        """Record one control step (called AFTER the physics step).

        ``q``/``qd`` are the raw simulator vectors; ``a`` is the 31-D
        contract-order upper-body command.  ``torque`` is the measured
        generalized actuator force (``qfrc_actuator``); ``torque_limit`` the
        per-DoF actuator range (read-only model limit).  ``contacts`` is the
        per-link dump produced by hmg_harness.obs (geom/body names, normal,
        distance, penetration, contact force); it is kept at events and every
        ``contact_every`` steps, never used to drive physics.
        """
        n_ev0 = len(self.events)
        self.t.append(float(t))
        self.q.append(np.asarray(q, np.float32))
        self.qd.append(np.asarray(qd, np.float32))
        self.a.append(np.asarray(a, np.float32))
        self.tau.append(None if torque is None
                        else np.asarray(torque, np.float32))
        if obj_pos is None:
            self.obj.append(None)
        else:
            self.obj.append(np.asarray(obj_pos, float).copy())
        self.obj_quat.append(None if obj_quat is None
                             else np.asarray(obj_quat, float).copy())
        if obj_aabb is None:
            self.obj_lo.append(None)
            self.obj_hi.append(None)
        else:
            self.obj_lo.append(np.asarray(obj_aabb[0], float).copy())
            self.obj_hi.append(np.asarray(obj_aabb[1], float).copy())
        self.base_z.append(None if base_z is None else float(base_z))
        self.tilt.append(float(tilt))
        self.hand_n.append(None if hand_contacts is None
                           else int(hand_contacts))
        self.nav.append(None if nav is None else np.asarray(nav, float).copy())
        self.height.append(None if height is None else float(height))
        if self._base0 is None and base_z is not None:
            self._base0 = float(base_z)
        # joint-limit margin
        if self.limits is not None:
            lo, hi = self.limits
            m = float(np.min(np.minimum(np.asarray(q) - lo, hi - np.asarray(q))))
            self.max['lim_margin'] = min(self.max['lim_margin'], m)
            if m < -1e-3:            # a real violation, not the reset pose
                self._fire('JOINT_LIMIT', t, {'margin': m})
        # command discontinuity
        if len(self.a) > 1:
            j = float(np.abs(self.a[-1] - self.a[-2]).max())
            self.max['act_jump'] = max(self.max['act_jump'], j)
            if j > self.th['act_jump']:
                self._fire('ACTION_JUMP', t, {'jump': j})
        if base_z is not None and self._base0 is not None:
            drop = self._base0 - float(base_z)
            self.max['base_drop'] = max(self.max['base_drop'], drop)
            if drop > self.th['base_drop']:
                self._fire('BASE_DROP', t, {'drop': drop})
        self.max['tilt'] = max(self.max['tilt'], abs(float(tilt)))
        if abs(float(tilt)) > self.th['tilt']:
            self._fire('TILT', t, {'tilt': float(tilt)})
        if hand_contacts is not None:
            self.max['hand_contacts'] = max(self.max['hand_contacts'],
                                            int(hand_contacts))
            if hand_contacts > 0:
                self._had_contact = True
            elif self._had_contact:
                self._fire('HAND_CONTACT_LOSS', t, {'n': 0})
        if obj_pos is not None and len(self.extra) > 0:
            prev = self.extra[-1].get('obj')
            if prev is not None:
                slip = float(np.linalg.norm(np.asarray(obj_pos) - prev))
                self.max['slip'] = max(self.max['slip'], slip)
                if slip > self.th['slip']:
                    self._fire('OBJECT_SLIP', t, {'dpos': slip})
        # measured torque vs the actuator's own range (model limits, read-only)
        if torque is not None:
            tq = np.asarray(torque, float)
            amax = float(np.abs(tq).max()) if tq.size else 0.
            self.max['torque'] = max(self.max.get('torque', 0.), amax)
            if torque_limit is not None:
                lim = np.asarray(torque_limit, float)
                if lim.shape == tq.shape:
                    ok = lim > 0
                    if ok.any():
                        frac = np.where(ok, np.abs(tq) / np.where(ok, lim, 1.),
                                        0.)
                        i = int(np.argmax(frac))
                        self.max['torque_frac'] = max(
                            self.max.get('torque_frac', 0.), float(frac[i]))
                        if frac[i] >= self.th['torque_frac']:
                            self._fire('TORQUE_SATURATION', t,
                                       {'tau': float(tq[i]),
                                        'limit': float(lim[i]),
                                        'frac': float(frac[i]),
                                        'dof': int(i)})
        # contact bookkeeping (evidence only; never changes the sim)
        ncon = None if contacts is None else len(contacts)
        if contacts is not None:
            ncon = len(contacts)
            if max_penetration is None and contacts:
                # only contacts that touch the task object count; foot/floor
                # penetration during normal walking is not a failure signal.
                max_penetration = max(
                    [float(c.get('penetration', 0.)) for c in contacts
                     if c.get('obj_contact')], default=0.)
            if max_penetration_all is None and contacts:
                max_penetration_all = max(
                    float(c.get('penetration', 0.)) for c in contacts)
            mp = 0. if max_penetration is None else float(max_penetration)
            mpa = (0. if max_penetration_all is None
                   else float(max_penetration_all))
            self.max['penetration'] = max(self.max.get('penetration', 0.), mp)
            self.max['penetration_all'] = max(
                self.max.get('penetration_all', 0.), mpa)
            self.max['ncon'] = max(self.max.get('ncon', 0), ncon)
            if mp > self.th['penetration']:
                self._fire('CONTACT_PENETRATION', t, {'penetration': mp})
        self.contact_n.append(ncon)
        # snapshots: periodic + every step that produced a new event
        if contacts is not None and (self._step % self.contact_every == 0
                                     or len(self.events) > n_ev0):
            self.contact_snapshots.append({
                't': float(t), 'step': int(self._step), 'ncon': int(ncon),
                'items': list(contacts[:self.max_contacts])})
        # attach the exact contact dump to events fired at this step
        for ev in self.events[n_ev0:]:
            if contacts is not None:
                ev['contacts'] = list(contacts[:self.max_contacts])
                ev['ncon'] = int(ncon)
        if progress is not None:
            if self._prog is not None and abs(progress - self._prog) < 1e-6:
                self._still += 1
            else:
                self._still = 0
            self._prog = progress
            if self._still == self.th['hold_steps']:
                self._fire('NO_PROGRESS', t, {'steps': self._still})
        self.extra.append({'obj': None if obj_pos is None
                           else np.asarray(obj_pos, float).copy()})
        self._step += 1

    def _fire(self, kind, t, detail):
        last = self._last_fire.get(kind)
        if last is not None and (t - last) < 0.4:   # debounce
            return
        self._last_fire[kind] = t
        self.events.append({'kind': kind, 't': float(t),
                            'detail': {k: float(v) for k, v in detail.items()}})

    # ---- on-demand evidence -------------------------------------------
    def _event(self, kind=None, index=0):
        """Resolve an event: (global_index, event) or (None, None)."""
        if kind is None:
            if 0 <= int(index) < len(self.events):
                i = int(index)
                return i, self.events[i]
            return None, None
        matches = [(i, e) for i, e in enumerate(self.events)
                   if e['kind'] == kind]
        if not matches:
            return None, None
        k = int(index)
        if k < 0:
            k = len(matches) + k
        if not (0 <= k < len(matches)):
            return None, None
        return matches[k]

    def window(self, kind=None, index=0, pad=0.2, max_samples=200,
               fields=None):
        """Sampled per-step arrays around one event (JSON-safe lists).

        ``kind`` selects the first event of that kind unless ``index`` picks
        another occurrence (negative counts from the end).  ``kind=None``
        with an integer ``index`` selects the index-th event of ANY kind.
        The window is [t_event - pad, t_event + pad], decimated to at most
        ``max_samples`` rows.  Returns the event (with residual detail and
        its contact dump), global index, time range and requested arrays.
        """
        i, ev = self._event(kind, index)
        if ev is None:
            return {'event': None, 'n_events': len(self.events),
                    'kind': kind, 'index': index}
        t = np.asarray(self.t, float)
        lo_t, hi_t = ev['t'] - float(pad), ev['t'] + float(pad)
        sel = np.where((t >= lo_t) & (t <= hi_t))[0]
        if sel.size == 0:
            j = int(np.argmin(np.abs(t - ev['t'])))
            sel = np.array([j])
        stride = max(1, int(np.ceil(sel.size / max(1, int(max_samples)))))
        sel = sel[::stride]
        want = list(fields) if fields else list(TRACE_FIELDS)
        bad = [f for f in want if f not in TRACE_FIELDS]
        if bad:
            return {'event': ev, 'event_index': i, 'error':
                    'unknown fields %s (choose from %s)'
                    % (bad, list(TRACE_FIELDS))}
        out = {'event': ev, 'event_index': int(i), 'residual': ev['detail'],
               't_event': ev['t'], 'pad': float(pad), 'stride': stride,
               'n_samples': int(sel.size), 'fields': want,
               'n_events': len(self.events)}
        for f in want:
            out[f] = self._slice(f, sel)
        return out

    def _slice(self, field, sel):
        if field == 't':
            return np.round(np.asarray(self.t, float)[sel], 5).tolist()
        src = getattr(self, field if field != 'act' else 'a')
        if field == 'act':
            src = self.a
        vals = []
        for j in sel:
            v = src[int(j)]
            if v is None:
                vals.append(None)
            else:
                arr = np.round(np.asarray(v, float), 5)
                vals.append(arr.tolist())
        return vals

    # ---- persistence ---------------------------------------------------
    def save(self, success, safety_ok, extra=None):
        """Write the trace + the coordinator-only summary + the actor-safe
        events view.  Returns the coordinator-only summary dict exactly in
        the historical shape plus the new trace fields."""
        trace_rel = 'trace.npz'
        arrays = self._arrays()
        np.savez_compressed(os.path.join(self.dir, trace_rel), **arrays)
        # per-link contact snapshots as JSON lines (actor-readable evidence)
        with open(os.path.join(self.dir, 'contacts.jsonl'), 'w') as f:
            for snap in self.contact_snapshots:
                f.write(json.dumps(snap) + '\n')
        earliest = self.events[:3]
        summary = dict(self.meta, success=bool(success),
                       safety_ok=bool(safety_ok), n_steps=len(self.t),
                       duration=self.t[-1] if self.t else 0.,
                       earliest_events=earliest,
                       maxima={k: round(float(v), 5) for k, v in self.max.items()},
                       trace=trace_rel,
                       contacts_log='contacts.jsonl',
                       n_contacts_snapshots=len(self.contact_snapshots))
        if extra:
            summary.update(extra)
        with open(os.path.join(self.dir, 'episode_summary.json'), 'w') as f:
            json.dump(summary, f, indent=2)
        self._final = dict(extra or {})
        # actor-safe view: NEVER contains success / safety_ok / reward
        pub = self.public(ended_by=self._final.get('ended_by', 'finish'),
                          trace=os.path.join(self.dir, trace_rel))
        with open(os.path.join(self.dir, 'events.json'), 'w') as f:
            json.dump(pub, f, indent=1)
        return summary

    def _arrays(self):
        """npz arrays; fields absent from every step are dropped."""
        def arr(name):
            vals = getattr(self, name)
            if not vals or all(v is None for v in vals):
                return None
            first = next(v for v in vals if v is not None)
            if np.isscalar(first) or np.asarray(first).ndim == 0:
                return np.asarray([np.nan if v is None else v
                                   for v in vals], float)
            shape = np.asarray(first).shape
            return np.asarray([np.full(shape, np.nan) if v is None
                               else np.asarray(v, float) for v in vals])
        out = {'t': np.asarray(self.t, float)}
        for name in ('q', 'qd', 'a', 'tau', 'obj', 'obj_quat', 'obj_lo',
                     'obj_hi', 'base_z', 'tilt', 'hand_n', 'contact_n',
                     'nav', 'height'):
            a = arr(name)
            key = 'act' if name == 'a' else name
            if a is not None:
                out[key] = a
        return out

    def public(self, ended_by='live', trace=None):
        """Actor-visible episode view.  Explicitly built from safe fields:
        success / safety_ok can never leak through this path."""
        counts = {}
        for e in self.events:
            counts[e['kind']] = counts.get(e['kind'], 0) + 1
        return {
            'task': self.meta.get('task'), 'seed': self.meta.get('seed'),
            'ended_by': ended_by,
            'n_steps': len(self.t),
            'duration': round(float(self.t[-1]), 5) if self.t else 0.,
            'earliest_events': [self._public_event(e)
                                for e in self.events[:3]],
            'maxima': {k: (round(float(v), 5) if k != 'lim_margin' or v < 1e8
                           else None)
                       for k, v in self.max.items()},
            'counts': {'events': len(self.events), 'by_kind': counts,
                       'contact_snapshots': len(self.contact_snapshots),
                       'hand_contacts_last': (self.hand_n[-1]
                                              if self.hand_n else None),
                       'ncon_last': (self.contact_n[-1]
                                     if self.contact_n else None)},
            'trace': trace,
        }

    @staticmethod
    def _public_event(ev, max_contacts=16):
        out = {'kind': ev['kind'], 't': ev['t'],
               'residual': ev.get('detail', {})}
        if 'contacts' in ev:
            out['ncon'] = ev.get('ncon')
            out['contacts'] = ev['contacts'][:max_contacts]
        return out

    def snapshot(self, fields=None, stride=1, t0=None, t1=None):
        """Current buffers as JSON-safe lists (used by the trace tool and by
        a live ``events`` call when the episode is not finalized)."""
        t = np.asarray(self.t, float)
        sel = np.arange(len(t))
        if t0 is not None:
            sel = sel[t[sel] >= float(t0)]
        if t1 is not None:
            sel = sel[t[sel] <= float(t1)]
        stride = max(1, int(stride))
        sel = sel[::stride]
        want = list(fields) if fields else list(TRACE_FIELDS)
        out = {'fields': want, 'n_samples': int(sel.size),
               't_range': [float(t[sel[0]]), float(t[sel[-1]])]
               if sel.size else None}
        for f in want:
            if f not in TRACE_FIELDS:
                continue
            out[f] = self._slice(f, sel)
        return out
