#!/usr/bin/env python3
"""Orbit target following with obstacles on the track.

The flat-terrain ONNX policy provides locomotion/balance; a high-level planner
tracks the orbiting target (tangential feedforward + radial correction) and
steers around cubes that sit on the circular track.
"""
from __future__ import annotations

import argparse
import math
import os
import sys

import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import orbit_scene as scene  # noqa: E402
from walk_test import PolicyRunner  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "doc", "demo")
FPS = 30
WIDTH, HEIGHT = 1280, 720
FONT_BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
FONT_REG = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"

OMEGA = scene.SPEED / scene.RADIUS
CUBES = [
    (scene.RADIUS * math.cos(math.radians(deg)),
     scene.RADIUS * math.sin(math.radians(deg)))
    for deg in scene.OBSTACLE_ANGLES
]


def wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class Planner:
    """Velocity-command planner: follow the orbit, steer around cubes."""

    def __init__(self) -> None:
        self.command = np.zeros(3, dtype=np.float32)
        self.distance = 0.0
        self.target = np.zeros(2)

    def update(self, pos: np.ndarray, yaw: float, angle: float) -> np.ndarray:
        self.target = np.array([scene.RADIUS * math.cos(angle),
                                scene.RADIUS * math.sin(angle)])
        tangent = np.array([-math.sin(angle), math.cos(angle)]) * scene.SPEED
        error = self.target - pos
        self.distance = float(np.linalg.norm(error))

        standoff = 0.90
        if self.distance > 1e-6:
            radial = error / self.distance
            v_des = tangent + 0.90 * (self.distance - standoff) * radial
        else:
            v_des = tangent.copy()

        forward = np.array([math.cos(yaw), math.sin(yaw)])
        left = np.array([-math.sin(yaw), math.cos(yaw)])
        for cube in CUBES:
            away = pos - cube
            dist = float(np.linalg.norm(away))
            if dist > 1.05 or dist < 1e-4:
                continue
            to_obs = cube - pos
            cross = forward[0] * to_obs[1] - forward[1] * to_obs[0]
            side = -left if cross >= 0.0 else left
            weight = 1.0 - dist / 1.05
            v_des += 1.05 * weight * side + 0.45 * weight * (away / dist)

        speed = float(np.linalg.norm(v_des))
        if speed > 0.95:
            v_des *= 0.95 / speed
            speed = 0.95

        vx = float(np.clip(np.dot(v_des, forward), -0.20, 0.85))
        vy = float(np.clip(np.dot(v_des, left), -0.35, 0.35))
        if speed > 0.12:
            heading = math.atan2(v_des[1], v_des[0])
            yaw_error = wrap(heading - yaw)
        else:
            yaw_error = 0.0
        wz = float(np.clip(1.75 * yaw_error, -0.90, 0.90))
        # Feed-forward the orbit turn rate so the robot does not lag the circle.
        radius = float(np.linalg.norm(pos))
        wz = float(np.clip(wz + vx / max(0.7, radius), -0.95, 0.95))
        self.command = np.array([vx, vy, wz], dtype=np.float32)
        return self.command


def hud(image, t, lap, dist, command):
    frame = Image.fromarray(image).convert("RGBA")
    overlay = Image.new("RGBA", frame.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    bold = ImageFont.truetype(FONT_BOLD, 32)
    mid = ImageFont.truetype(FONT_BOLD, 24)
    small = ImageFont.truetype(FONT_REG, 19)

    draw.rounded_rectangle((28, 22, 940, 126), radius=14, fill=(8, 12, 18, 200))
    draw.text((48, 32), "Unitree Go2 — orbit target following", font=bold,
              fill=(245, 245, 245, 255))
    draw.text((48, 78),
              f"target orbit R={scene.RADIUS:.1f} m · {scene.SPEED:.2f} m/s "
              f"· {2 * math.pi * scene.RADIUS / scene.SPEED:.1f} s/lap",
              font=small, fill=(180, 195, 210, 255))
    draw.text((48, 100), "flat ONNX policy · planner: feedforward + obstacle steer",
              font=small, fill=(130, 145, 160, 255))

    draw.rounded_rectangle((28, 146, 400, 214), radius=12, fill=(40, 120, 200, 225))
    draw.text((48, 156), f"LAP {lap}", font=mid, fill=(255, 255, 255, 255))
    draw.text((48, 186), f"target distance {dist:4.2f} m", font=small,
              fill=(235, 240, 245, 235))

    draw.rounded_rectangle((28, 620, 1252, 700), radius=12, fill=(8, 12, 18, 185))
    draw.text((48, 632),
              f"vx {command[0]:+.2f} m/s   vy {command[1]:+.2f} m/s   "
              f"wz {command[2]:+.2f} rad/s",
              font=small, fill=(205, 215, 225, 255))
    draw.text((48, 660), f"t = {t:5.2f} s", font=small, fill=(140, 150, 160, 255))
    draw.text((720, 632), "obstacles: 4 cubes on the track", font=small,
              fill=(170, 185, 200, 255))
    return np.array(Image.alpha_composite(frame, overlay).convert("RGB"))


def title_frames():
    image = Image.new("RGB", (WIDTH, HEIGHT), (12, 18, 26))
    draw = ImageDraw.Draw(image)
    for y in range(HEIGHT):
        shade = int(12 + 16 * (y / HEIGHT))
        draw.line([(0, y), (WIDTH, y)], fill=(shade, shade + 6, shade + 14))
    title = ImageFont.truetype(FONT_BOLD, 50)
    sub = ImageFont.truetype(FONT_BOLD, 26)
    small = ImageFont.truetype(FONT_REG, 20)
    draw.text((90, 190), "Unitree Go2 — orbit target following", font=title,
              fill=(240, 244, 248))
    draw.text((90, 270), "Target circles a 2.2 m track at 0.60 m/s",
              font=sub, fill=(120, 200, 255))
    draw.text((90, 340), "4 obstacle cubes on the track · flat ONNX policy · MuJoCo",
              font=small, fill=(195, 208, 220))
    draw.text((90, 380), "Planner: tangential feedforward + radial correction + obstacle steering",
              font=small, fill=(195, 208, 220))
    draw.text((90, 620), "No new training: existing flat policy (iter 1999).",
              font=small, fill=(120, 132, 144))
    frame = np.array(image)
    return [frame.copy() for _ in range(int(1.8 * FPS))]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--onnx", default=os.path.join(ROOT, "demo", "policies", "flat.onnx"))
    parser.add_argument("--duration", type=float, default=28.0)
    parser.add_argument("--out", default=os.path.join(OUT, "go2-orbit-follow.mp4"))
    parser.add_argument("--probe", action="store_true")
    parser.add_argument("--no-title", action="store_true")
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(scene.build_scene())
    model.opt.timestep = 0.005
    model.vis.global_.offwidth = WIDTH
    model.vis.global_.offheight = HEIGHT
    data = mujoco.MjData(model)
    runner = PolicyRunner(args.onnx, model, data)
    runner.reset()
    data.qpos[0:2] = [0.0, 0.0]
    data.qpos[2] = 0.32
    mujoco.mj_forward(model, data)

    target_mocap = model.body("orbit_target").mocapid
    planner = Planner()
    angle = 0.0
    renderer = mujoco.Renderer(model, HEIGHT, WIDTH)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0.0, 0.0, 0.40]
    camera.distance = 5.2
    camera.azimuth = 90.0
    camera.elevation = -35.0

    control_every = int(round(0.02 / model.opt.timestep))
    frames = [] if not args.probe else None
    if frames is not None and not args.no_title:
        frames.extend(title_frames())
    next_render = 0.0
    step = 0
    probe_times = [2.0, 8.0, 16.0, 24.0]
    probe_index = 0
    next_log = 0.0
    finished = False
    try:
        while data.time < args.duration:
            if step % control_every == 0:
                pos = data.qpos[0:2].copy()
                yaw = math.atan2(2.0 * (data.qpos[3] * data.qpos[6]
                                        + data.qpos[4] * data.qpos[5]),
                                 1.0 - 2.0 * (data.qpos[5] ** 2 + data.qpos[6] ** 2))
                angle = (angle + OMEGA * 0.02) % (2.0 * math.pi)
                command = planner.update(pos, yaw, angle)
                runner.act(command)
                data.mocap_pos[target_mocap] = [
                    scene.RADIUS * math.cos(angle),
                    scene.RADIUS * math.sin(angle), scene.BALL_Z,
                ]
            runner.control()
            mujoco.mj_step(model, data)
            step += 1

            if data.time >= next_log:
                next_log += 1.0
                nearest = min(float(np.linalg.norm(data.qpos[0:2] - np.array(c)))
                              for c in CUBES)
                print(f"t={data.time:5.2f}  pos=({data.qpos[0]:5.2f},{data.qpos[1]:5.2f})  "
                      f"z={data.qpos[2]:.3f}  dist={planner.distance:4.2f}  "
                      f"cube={nearest:.2f}  "
                      f"cmd=({planner.command[0]:+.2f},{planner.command[1]:+.2f},"
                      f"{planner.command[2]:+.2f})", flush=True)

            if data.time >= next_render:
                next_render += 1.0 / FPS
                renderer.update_scene(data, camera=camera)
                lap = int(angle / (2.0 * math.pi)) + 1
                image = hud(renderer.render(), data.time, lap, planner.distance,
                            planner.command)
                if args.probe:
                    if probe_index < len(probe_times) and data.time >= probe_times[probe_index]:
                        Image.fromarray(image).save(f"/tmp/orbit_probe_{probe_index}.png")
                        print("probe", f"/tmp/orbit_probe_{probe_index}.png")
                        probe_index += 1
                else:
                    frames.append(image)
    finally:
        renderer.close()
        if not args.probe:
            os.makedirs(os.path.dirname(args.out), exist_ok=True)
            imageio.mimsave(args.out, frames, fps=FPS, quality=6, macro_block_size=None)
            print("wrote", args.out, "frames", len(frames))
            finished = True
    print("final z", float(data.qpos[2]))


if __name__ == "__main__":
    main()
