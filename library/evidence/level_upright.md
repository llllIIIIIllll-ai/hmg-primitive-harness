# Evidence: level_upright

Verbatim catalog entry as it stood before the 2026-11-10 compaction of
`PRIMITIVES.md`. The catalog keeps the decision-changing summary and points
here for proof detail (leaf counts, per-seed `t`, replay methodology,
refuted sub-claims, superseded diffs). Coordinates with
`library/SUPERSEDED.md` (retired variant hashes) and
`library/CORRECTIONS.md` (refuted promotion claims).

```
## level_upright(joints=(14,15), step=0.08, target=0.996, min_gain=0.001, max_tries=4, dur=0.35, reverse=2.0, sign_reversal=True, read_mode='geom', target_stop='ge', out='two')
- source hash: 869195fcbf42c7f12d416a77fd3c04080c1f9627e82eced8953ece04844ed238 (work/tracks/06_drill_pnp/actor_extract/skills/level_upright.py; the identical bytes also sit at `merge_round/level_upright.py`, where the verification evidence was produced)
- removes: the union of its superseded variants -- a wrist rotation that makes the object LESS upright (keep-only-if-it-gained, rejected probe undone) and the residual tilt about a second wrist axis (coordinate ascent with a one-shot direction reversal).
- applies when: a grasp is held and `nudge_wrist_joint` is registered; one or two wrist axes dominate the tilt. `read_mode` and `target_stop` are PHYSICAL parameters, not diagnostics: 'nudge' uses the nudge's own z_up and adds no read, 'geom' reads explicitly after every probe, opposite probe and restore; 'gt' probes again at exact target equality, 'ge' stops. CALLERS MUST MIGRATE KWARG NAMES: `joint`->`joints`, `delta`->`step`; `**kw` swallows the old names, so an unmigrated call silently runs the TWO-AXIS DEFAULTS.
- contract: pre: measured uprightness below target -> post: one read, then per joint a signed step; each probe is one `nudge_wrist_joint`; accepted if z > z0 + min_gain; on failure either undo (sign_reversal False) or opposite probe then restore; final read. Returns `{log, zup_after}` ('two') or `{zup_before, zup_after}` ('single'). Side effects: reads as per read_mode; the only commands are the nudge calls.
- verified scope: 06_drill_pnp, drill, seeds 0-9 in BOTH modes (stage replay: 0 differing leaves, identical geom/command counts, max delta 0, including the single-axis undo branch on 1,7,9 and the two-axis reversal branch) and seeds 0-9 at the whole-composition level: the canonical `actor_extract/compose.py` b765a328…7236 (old bytes preserved at `compose_superseded.py` bd13f59a…4029) vs the pinned promoted program 455a237a…358f, **0 differing leaves on every seed**, identical t (31.36/23.70/31.46/32.32/31.64/34.66/33.80/29.12/32.04/32.64). Inherited narrowings: it never guarantees reaching target. In the REAL composition the two-axis call on seeds 1 and 9 returns an EMPTY accepted log with z_up BELOW its entry (0.9756 -> 0.9657; 0.9808 -> 0.9649); an empty log means no ACCEPTED probe, not no probe -- probes are issued and reverted, which is why the exit can sit below entry. Per-seed accepted entries are 2,0,3,4,2,3,1,3,5,0 for seeds 0-9. The per-skill stage replay does NOT show this shape because it enters at a different state (z_up 0.9084); the whole-program equality is the binding evidence.
- used by: 06_drill_pnp `merge_round/compose_merged.py` (two calls with the mapped parameters).
```
