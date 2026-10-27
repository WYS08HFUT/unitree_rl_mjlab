# Go2 runtime posture demo

A 9-second, 1280×720 / 30 fps demo of the runtime skeleton's posture state
machine: **Stand → Lie → Stand → Software E-stop (Passive, latched)**.

The behavior timeline is produced by the **compiled C++ `runtime::Runtime`**
(`demo/runtime_timeline.cpp`), not by a Python re-implementation. The renderer
replays that timeline in MuJoCo 3.5 and drives the same scripted posture
targets the deploy config uses (`FixStand`: `[0, 0.8, -1.5]`,
`LieDown`: `[0, 1.45, -2.70]`).

![poster](go2-runtime-skeleton-poster.png)

## Reproduce

```bash
# 1. Build the behavior-timeline harness (pure core, no SDK)
g++ -std=c++17 -I deploy/include demo/runtime_timeline.cpp -o /tmp/runtime_timeline
/tmp/runtime_timeline > /tmp/runtime_timeline.csv

# 2. Render the video (MuJoCo + EGL, uses the project .venv)
MUJOCO_GL=egl .venv/bin/python demo/render_demo.py \
  --timeline /tmp/runtime_timeline.csv \
  --out doc/demo/go2-runtime-skeleton.mp4
```

`demo/render_demo.py --probe` writes four PNG stills to `/tmp` instead of a
video.

## Scope / honesty

- Posture targets are **scripted joint trajectories**, not a trained policy.
- `Velocity` is disabled in this round, so no walking is shown.
- This is **not** `unitree_mujoco` + `go2_ctrl`: that path needs GLFW, DDS and
  a display, none of which are available on the build machine. The demo shows
  the runtime's behavior transitions and the config's posture trajectories in
  a direct MuJoCo simulation.
- The E-stop segment renders `kPassive` as zero position gain; the real
  `State_Passive` uses damping-only gains, which is the same qualitative
  collapse.

## Target following + obstacle avoidance (flat → rough)

Video: `doc/demo/go2-flat-to-rough-target.mp4` (34.6 s, 1280×720 @ 30 fps).

Two segments, both driven by the exported ONNX policy at 50 Hz in MuJoCo:

| Stage | Policy | Terrain | Scene |
|---|---|---|---|
| 1 | `Unitree-Go2-Flat`, 1999 iters | flat | moving target + 4 obstacles |
| 2 | `Unitree-Go2-Rough-NoScan`, 3000 iters (resumed from flat) | smooth 0.25 m bumps | moving target + 4 obstacles |

The high-level planner keeps a 1.2 m standoff from a target moving on a square
loop and adds a potential-field repulsion from the known obstacle positions.
Balance and gait come from the PPO policy; the planner only outputs `(vx, vz)`.
**The planner uses ground-truth target/obstacle positions — there is no onboard
perception in this demo.**

### Training

```bash
# Stage 1: flat
WANDB_MODE=offline .venv/bin/python scripts/train.py Unitree-Go2-Flat \
  --env.scene.num-envs=4096 --agent.max-iterations=2000

# Stage 2: rough terrain without terrain perception (matches deploy.yaml)
# Resumes from the flat checkpoint; same experiment name keeps one log root.
WANDB_MODE=offline .venv/bin/python scripts/train.py Unitree-Go2-Rough-NoScan \
  --env.scene.num-envs=4096 --agent.max-iterations=3000 \
  --agent.experiment-name=go2_velocity \
  --agent.resume=True \
  --agent.load-run=<flat-run-dir> --agent.load-checkpoint=model_1999.pt
```

`Unitree-Go2-Rough-NoScan` is a repo-local task variant that keeps the rough
terrain generator but removes `height_scan`/`terrain_scan`, so its actor
observation is the 47-dim deploy contract (`deploy.yaml`) and the policy can be
run in plain MuJoCo or on the robot without a terrain sensor.

Exported policies used for the video live in `demo/policies/`:

- `flat.onnx` — `Unitree-Go2-Flat` (iter 1999)
- `rough.onnx` — `Unitree-Go2-Rough-NoScan` (iteration 3000)

### Rendering

```bash
.venv/bin/python demo/go2_demo.py \
  --onnx demo/policies/flat.onnx --flat --duration 16 \
  --stage "STAGE 1 · FLAT TERRAIN" --out doc/demo/stage1-flat.mp4

.venv/bin/python demo/go2_demo.py \
  --onnx demo/policies/rough.onnx --duration 16 \
  --stage "STAGE 2 · ROUGH TERRAIN" --out doc/demo/stage2-rough.mp4

.venv/bin/python demo/compose_video.py \
  --segments doc/demo/stage1-flat.mp4 doc/demo/stage2-rough.mp4 \
  --out doc/demo/go2-flat-to-rough-target.mp4
```

### Known limitations

- No onboard perception: target and obstacles are privileged positions.
- The rough policy is proprioception-only (no height scan), trained on mjlab's
  rough terrain; the demo bumps are smoothed heightfield waves, not stairs.
- Speed tracking is imperfect (rough policy ≈0.4–0.5 m/s average for 0.6 m/s
  commands), and the robot occasionally pauses to turn.
- This is a MuJoCo/ONNX demo, not the `unitree_mujoco + go2_ctrl` DDS stack.

## Orbit target following with obstacles on the track

Video: `doc/demo/go2-orbit-follow.mp4` (31.8 s, 1280×720 @ 30 fps).

- Target sphere orbits the origin at **R = 2.2 m, 0.60 m/s** (23.0 s/lap), floating at 0.9 m.
- **4 obstacle cubes** sit on the track at 50°/135°/220°/305°.
- The robot starts at the center and follows the target for ~1.3 laps.
- **No new training**: this uses the existing flat policy (`demo/policies/flat.onnx`,
  iteration 1999). The planner is `demo/orbit_demo.py`:
  tangential velocity feedforward + radial correction to a 0.9 m standoff +
  tangential steering around cubes.
- Scene preview assets: `doc/demo/orbit-scene-top.png`,
  `doc/demo/orbit-scene-persp.png`, `doc/demo/orbit-scene-preview.mp4`
  (`demo/orbit_scene.py`, no policy loaded).

Verification (30 s run): body height stayed **0.307–0.326 m**, closest approach
to a cube was **0.62 m**, target distance settled at **0.6–1.4 m** after the
initial 2.2 m approach.

```bash
.venv/bin/python demo/orbit_demo.py \
  --onnx demo/policies/flat.onnx --duration 30 \
  --out doc/demo/go2-orbit-follow.mp4
```

Known limitations: the planner uses true target/cube positions (no onboard
perception), the target is a floating marker, the ground is flat, and this is a
MuJoCo/ONNX demo rather than the `unitree_mujoco + go2_ctrl` stack.
