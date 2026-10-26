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
