# Go2 runtime skeleton

## Build

The SDK is installed to a local prefix (sudo is unavailable in this container):

```bash
git clone --depth 1 https://github.com/unitreerobotics/unitree_sdk2.git ~/unitree_sdk2
cmake -S ~/unitree_sdk2 -B ~/unitree_sdk2/build \
  -DBUILD_EXAMPLES=OFF -DCMAKE_INSTALL_PREFIX=$HOME/unitree_sdk2/install
cmake --build ~/unitree_sdk2/build -j
cmake --install ~/unitree_sdk2/build

P=$HOME/unitree_sdk2/install
cmake -S deploy/robots/go2 -B deploy/robots/go2/build \
  -DCMAKE_CXX_FLAGS="-I$P/include -I$P/include/ddscxx" \
  -DCMAKE_EXE_LINKER_FLAGS="-L$P/lib -Wl,-rpath,$P/lib"
cmake --build deploy/robots/go2/build -j
```

Run (on a machine with a robot or unitree_mujoco, gamepad optional):

```bash
cd deploy/robots/go2/build && ./go2_ctrl --network=lo
```

## Keyboard

| Key | Action |
|---|---|
| `1` | stand |
| `2` | lie down |
| `0` | passive |
| `3` | velocity (disabled until a policy is installed) |
| `x` | software E-stop (latch) |
| `r` | re-arm (needs 0.5 s of safe state) |
| `w/s` | vx +/- 0.1 m/s |
| `a/d` | vy +/- 0.1 m/s |
| `q/e` | wz +/- 0.2 rad/s |
| `space` | zero command |
| `z` | toggle slow mode (x0.4) |

## Gamepad (DSL)

| Action | Expression |
|---|---|
| stand | `LT + up.on_pressed` |
| lie | `LT + down.on_pressed` |
| passive | `LT + B.on_pressed` |
| estop | `RT + B.on_pressed` |
| rearm | `LT + right.on_pressed` |

## Safety

- DEGRADED (tilt > 0.35 rad, gyro > 3.5 rad/s, joint vel > 30 rad/s): command zeroed, posture held, auto-recovers after 0.5 s.
- CRITICAL (tilt > 0.60 rad, gyro > 6 rad/s, joint vel > 50 rad/s, NaN/jump, E-stop): command zeroed, transition to Passive, latched.
- Re-arm only in Passive after 0.5 s of OK; then press `1` (stand) explicitly.
- Thresholds live under `Runtime.safety` in `config/config.yaml`.

## Tests

```bash
cmake -S deploy/tests -B build/tests
cmake --build build/tests -j
ctest --test-dir build/tests --output-on-failure
```
