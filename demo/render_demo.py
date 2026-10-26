#!/usr/bin/env python3
"""Render the Go2 posture-state demo video.

The behavior timeline is produced by the compiled C++ `runtime::Runtime`
(`demo/runtime_timeline.cpp`). This script only replays that timeline in
MuJoCo, tracking the same stand/lie posture targets the deploy config uses
(`FixStand`, `LieDown`), and draws a simple HUD.

Usage:
    MUJOCO_GL=egl .venv/bin/python demo/render_demo.py \
        --timeline /tmp/runtime_timeline.csv \
        --out doc/demo/go2-runtime-skeleton.mp4
"""
from __future__ import annotations

import argparse
import csv
import os

import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE = os.path.join(
    ROOT, "src", "assets", "robots", "unitree_go2", "xmls", "scene_go2.xml"
)
FONT_BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
FONT_REG = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"

PASSIVE, STAND, LIE = 0, 1, 2
STAND_POSE = np.array([0.0, 0.8, -1.5])
LIE_POSE = np.array([0.0, 1.45, -2.70])
KP = np.array([60.0, 80.0, 80.0])
KD = np.array([5.0, 4.0, 4.0])
PASSIVE_KD = 3.0
FPS = 30
DURATION = 9.0
WIDTH, HEIGHT = 1280, 720


def load_timeline(path: str):
    rows = []
    with open(path, newline="") as handle:
        for row in csv.reader(handle):
            if len(row) != 4:
                continue
            rows.append((float(row[0]), int(row[1]), int(row[2]), int(row[3])))
    rows.sort(key=lambda item: item[0])
    return rows


def at_time(rows, t: float):
    lo, hi = 0, len(rows) - 1
    while lo < hi:
        mid = (lo + hi + 1) // 2
        if rows[mid][0] <= t:
            lo = mid
        else:
            hi = mid - 1
    return rows[lo]


class Posture:
    """Mirrors the FSM's scripted posture transitions (FixStand/LieDown)."""

    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.behavior = None
        self.move = None
        self.joint_qpos = []
        self.joint_dof = []
        for index in range(model.nu):
            joint = int(model.actuator_trnid[index, 0])
            self.joint_qpos.append(int(model.jnt_qposadr[joint]))
            self.joint_dof.append(int(model.jnt_dofadr[joint]))
        self.ctrl_lo = model.actuator_ctrlrange[:, 0].copy()
        self.ctrl_hi = model.actuator_ctrlrange[:, 1].copy()

    def reset(self):
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        mujoco.mj_forward(self.model, self.data)
        self.behavior = None
        self.move = None

    def joints(self):
        return self.data.qpos[self.joint_qpos].copy()

    def update_behavior(self, t: float, behavior: int):
        if behavior == self.behavior:
            return
        self.behavior = behavior
        if behavior in (STAND, LIE):
            pose = STAND_POSE if behavior == STAND else LIE_POSE
            duration = 2.0 if behavior == STAND else 1.5
            self.move = (t, self.joints(), pose, duration)
        else:
            self.move = None

    def target(self, t: float):
        if self.move is None:
            return self.joints()
        start_t, q_start, pose, duration = self.move
        alpha = float(np.clip((t - start_t) / duration, 0.0, 1.0))
        alpha = alpha * alpha * (3.0 - 2.0 * alpha)  # smoothstep, like a lerp
        target = (1.0 - alpha) * q_start + alpha * np.tile(pose, 4)
        return target

    def torques(self, t: float):
        if self.behavior == PASSIVE:
            dq = self.data.qvel[self.joint_dof]
            return np.clip(-PASSIVE_KD * dq, self.ctrl_lo, self.ctrl_hi)
        q = self.joints()
        dq = self.data.qvel[self.joint_dof]
        target = self.target(t)
        kp = np.tile(KP, 4)
        kd = np.tile(KD, 4)
        tau = kp * (target - q) - kd * dq
        return np.clip(tau, self.ctrl_lo, self.ctrl_hi)


def state_style(behavior: int, latched: bool):
    if behavior == PASSIVE and latched:
        return "E-STOP  ·  PASSIVE (latched)", (231, 76, 60), "rearm required"
    if behavior == PASSIVE:
        return "PASSIVE", (110, 110, 110), "damping"
    if behavior == LIE:
        return "LIE  ·  LieDown", (52, 152, 219), "scripted posture"
    if behavior == STAND:
        return "STAND  ·  FixStand", (46, 204, 113), "scripted posture"
    return "VELOCITY", (241, 196, 15), "policy"


def draw_hud(image: np.ndarray, t: float, behavior: int, latched: bool) -> np.ndarray:
    frame = Image.fromarray(image).convert("RGBA")
    overlay = Image.new("RGBA", frame.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    bold = ImageFont.truetype(FONT_BOLD, 34)
    mid = ImageFont.truetype(FONT_BOLD, 26)
    small = ImageFont.truetype(FONT_REG, 18)

    draw.rounded_rectangle((28, 22, 760, 118), radius=14, fill=(8, 12, 18, 190))
    draw.text((48, 34), "Unitree Go2 — runtime posture states", font=bold,
              fill=(245, 245, 245, 255))
    draw.text((48, 80), "behavior timeline from compiled C++ runtime::Runtime · MuJoCo 3.5",
              font=small, fill=(170, 180, 190, 255))

    label, color, detail = state_style(behavior, latched)
    draw.rounded_rectangle((28, 140, 520, 206), radius=12, fill=(*color, 220))
    draw.text((48, 150), label, font=mid, fill=(255, 255, 255, 255))
    draw.text((48, 182), detail, font=small, fill=(240, 240, 240, 230))

    draw.rounded_rectangle((28, 636, 1252, 700), radius=12, fill=(8, 12, 18, 170))
    draw.text((48, 648),
              "posture targets from deploy config  ·  no policy  ·  Velocity disabled",
              font=small, fill=(185, 195, 205, 255))
    draw.text((48, 672), f"t = {t:5.2f} s", font=small, fill=(140, 150, 160, 255))

    bar_x0, bar_x1, bar_y = 300, 1232, 678
    draw.rounded_rectangle((bar_x0, bar_y, bar_x1, bar_y + 8), radius=4,
                           fill=(50, 58, 68, 255))
    progress = float(np.clip(t / DURATION, 0.0, 1.0))
    draw.rounded_rectangle((bar_x0, bar_y,
                            bar_x0 + (bar_x1 - bar_x0) * progress, bar_y + 8),
                           radius=4, fill=(*color, 255))

    return np.array(Image.alpha_composite(frame, overlay).convert("RGB"))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeline", default="/tmp/runtime_timeline.csv")
    parser.add_argument("--out", default=os.path.join(ROOT, "doc", "demo",
                                                      "go2-runtime-skeleton.mp4"))
    parser.add_argument("--probe", action="store_true",
                        help="write four PNG stills instead of a video")
    args = parser.parse_args()

    rows = load_timeline(args.timeline)
    model = mujoco.MjModel.from_xml_path(SCENE)
    model.opt.timestep = 0.002
    model.vis.global_.offwidth = WIDTH
    model.vis.global_.offheight = HEIGHT
    data = mujoco.MjData(model)
    posture = Posture(model, data)
    posture.reset()

    renderer = mujoco.Renderer(model, HEIGHT, WIDTH)
    camera = mujoco.MjvCamera()
    camera.distance = 1.75
    camera.azimuth = 135.0
    camera.elevation = -16.0

    sim_dt = model.opt.timestep
    clock = {"t": 0.0}

    def advance_to(t_end: float) -> None:
        while clock["t"] < t_end - 1e-9:
            _, behavior, latched, _ = at_time(rows, clock["t"])
            posture.update_behavior(clock["t"], behavior)
            data.ctrl[:] = posture.torques(clock["t"])
            mujoco.mj_step(model, data)
            clock["t"] += sim_dt

    def render_at(t: float) -> np.ndarray:
        _, behavior, latched, _ = at_time(rows, t)
        camera.lookat[0] = float(data.qpos[0])
        camera.lookat[1] = float(data.qpos[1])
        camera.lookat[2] = 0.30
        renderer.update_scene(data, camera=camera)
        return draw_hud(renderer.render(), t, behavior, bool(latched))

    os.makedirs(os.path.dirname(args.out), exist_ok=True)

    if args.probe:
        probe_times = [0.6, 3.4, 6.9, 8.4]
        for index, probe_t in enumerate(probe_times):
            advance_to(probe_t)
            path = f"/tmp/demo_probe_{index}.png"
            Image.fromarray(render_at(probe_t)).save(path)
            print("wrote", path)
        return

    writer = imageio.get_writer(
        args.out, fps=FPS, codec="libx264", quality=8,
        pixelformat="yuv420p", macro_block_size=None)
    try:
        for frame_index in range(int(DURATION * FPS)):
            frame_t = (frame_index + 1) / FPS
            advance_to(frame_t)
            writer.append_data(render_at(frame_t))
    finally:
        writer.close()

    if not args.probe:
        print("wrote", args.out)


if __name__ == "__main__":
    main()
