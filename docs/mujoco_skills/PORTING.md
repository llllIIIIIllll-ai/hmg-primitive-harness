# Porting the MuJoCo skills into HMG

Runtime pin: `mujoco==3.2.6` (`requirements.txt`). The guides are from upstream
`main` at commit `7252110fb3060fee5fbef9f341f36341e6b02a87`.

| Guide | Status in HMG | Notes |
|---|---|---|
| `python/` | **Applies** | Core simulation lifecycle. HMG uses `MjModel`/`MjData`, `mj_forward`, `mj_step`, named access and `bind()` in `hmg_tools.py` and `hmg_harness/`. Check API details against 3.2.6. |
| `rendering/` | **Applies** | `hmg_render.py` renders offscreen stills/mp4. Keep `MUJOCO_GL=egl` on headless hosts. |
| `spec_editing/` | Reference only | HMG loads the benchmark MJCF. Do not edit benchmark models or assets. |
| `accelerated/` | Not used | No MJX/MJWarp; the harness runs CPU MuJoCo, one server per task. |
| `deformables/` | Not used | No deformable bodies in the HMG tasks. |
| `parity/` | Conceptual | Useful testing ideas, but no parity test is wired into this harness. |
| `gui/` | Not used | Headless operation. |
| `studio/` | Not used | No Studio app or interactive viewer. |

## Porting rules

1. Do not change physics, reward, the success predicate, or model limits.
2. Use the pinned 3.2.6 API; if a guide uses a newer helper, find the 3.2.6
   equivalent or leave the guide as reference.
3. Do not add a code dependency on these Markdown files; they are documentation.
4. Keep the harness's observation/recording contract unchanged: tools and the
   event record are the only actor-visible feedback.
