# curl_fingers_onto_handle -- open before calling

Signature: curl_fingers_onto_handle(angle=1.3, dur=1.1)
- source hash: 7df7ca5df6260ff7a4d5c66c3cb374da3be8a53e7b4c56a0f4f37fb3687c7767 (06_drill_pnp/actor_extract/skills/curl_fingers_onto_handle.py)
- removes: trusting an open-loop curl -- a standoff a few mm off closes the fingers on air and every later phase gates on a false success.
- applies when: the grasping palm is at a standoff with the handle between the fingers.
- contract: pre: palm at the standoff, object at rest -> post: fingers at `angle` with the per-side hand-object contact counts MEASURED and returned (3-5 on the grasping side, 0 on the idle side, seeds 0-9). Side effects: one `close_grip` plus two object reads and two contact scans.
- verified scope: 06_drill_pnp, drill handle, seeds 0-9. Breaks when the handle is outside the finger span at the standoff depth -- 08's sibling skill closed around a 20 mm-misaligned handle and the object left the gripper later, so the contact count, not the angle, is the contract.
- used by: 06_drill_pnp composition.
- CAUTION: the measured contact count, not the commanded angle, is the contract -- a misaligned handle closes on air.

Proof history: `library/evidence/curl_fingers_onto_handle.md`.
