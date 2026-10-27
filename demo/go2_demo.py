#!/usr/bin/env python3
"""Target-follow + obstacle-avoidance demo for a trained Go2 policy.

The policy is the exported ONNX (deploy.yaml contract). A high-level
controller steers toward a moving target and around known obstacles; the
policy provides balance and locomotion on flat or rough terrain.

Usage:
  .venv/bin/python demo/go2_demo.py --onnx <policy.onnx> --out doc/demo/xxx.mp4
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
from walk_test import PolicyRunner  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE = os.path.join(ROOT, "src", "assets", "robots", "unitree_go2", "xmls", "scene_go2.xml")
MESH_DIR = os.path.join(ROOT, "src", "assets", "robots", "unitree_go2", "xmls", "assets")

FPS = 30
WIDTH, HEIGHT = 1280, 720
FONT_BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
FONT_REG = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"

OBSTACLES = [  # (x, y, half_size, height)
    (0.90, 1.80, 0.12, 0.45),
    (-1.80, 0.90, 0.13, 0.50),
    (-0.90, -1.80, 0.12, 0.45),
    (1.80, -0.90, 0.13, 0.50),
]
WAYPOINTS = [
    (1.8, 1.8),
    (-1.8, 1.8),
    (-1.8, -1.8),
    (1.8, -1.8),
]


def terrain_height(x: float, y: float) -> float:
    """Smooth bump field, flat near the origin where the robot starts."""
    r = math.hypot(x, y)
    envelope = float(np.clip((r - 0.9) / 1.6, 0.0, 1.0))
    envelope = envelope * envelope * (3.0 - 2.0 * envelope)
    h = (0.105 * math.sin(1.9 * x + 0.4) * math.cos(1.5 * y)
         + 0.070 * math.sin(3.1 * y + 1.2)
         + 0.040 * math.cos(4.2 * x - 0.7))
    return max(0.0, envelope * h)


def make_heightmap(path: str, n: int = 128, radius: float = 4.0) -> None:
    data = np.zeros((n, n), dtype=np.uint8)
    for i in range(n):
        y = radius * (2.0 * i / (n - 1) - 1.0)
        for j in range(n):
            x = radius * (2.0 * j / (n - 1) - 1.0)
            data[i, j] = int(np.clip(terrain_height(x, y) / 0.25, 0.0, 1.0) * 255)
    Image.fromarray(data, mode="L").save(path)


def build_scene(rough: bool, heightmap: str) -> str:
    with open(SCENE) as handle:
        xml = handle.read()
    xml = xml.replace('meshdir="assets"', f'meshdir="{MESH_DIR}"')
    floor = ('<geom name="floor" size="0 0 0.05" type="plane" '
             'material="groundplane"/>')
    if rough:
        replacement = (
            '<geom name="floor" type="plane" size="8 8 0.1" pos="0 0 -0.4" '
            'material="groundplane"/>'
            f'<geom name="terrain" type="hfield" hfield="terrain" pos="0 0 0" '
            'material="groundplane"/>'
        )
        asset = (f'<hfield name="terrain" file="{heightmap}" size="4 4 0.25 0.01" '
                 'nrow="128" ncol="128"/>\n  ')
        xml = xml.replace("</asset>", asset + "</asset>", 1)
    else:
        replacement = floor
    assert floor in xml
    xml = xml.replace(floor, replacement)

    extra = []
    for index, (x, y, half, height) in enumerate(OBSTACLES):
        z = terrain_height(x, y) + height * 0.5
        extra.append(
            f'<geom name="obstacle_{index}" type="box" size="{half} {half} {height * 0.5}" '
            f'pos="{x} {y} {z}" rgba="0.85 0.42 0.12 1"/>'
        )
    extra.append(
        '<body name="target" mocap="true" pos="1.8 1.8 0.30">'
        '<geom name="target_geom" type="sphere" size="0.13" rgba="1 0.12 0.10 1" '
        'contype="0" conaffinity="0"/></body>'
    )
    xml = xml.replace("</worldbody>", "  " + "\n  ".join(extra) + "\n  </worldbody>", 1)
    path = "/tmp/go2_demo_scene.xml"
    with open(path, "w") as handle:
        handle.write(xml)
    return path


def yaw_from_quat(quat) -> float:
    w, x, y, z = quat
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class HighLevel:
    def __init__(self, target_xy):
        self.target = np.array(target_xy, dtype=float)
        self.waypoint = 1
        self.distance = 0.0
        self.command = np.zeros(3, dtype=np.float32)

    def update(self, pos_xy, yaw, dt):
        # Move the target along the waypoint path at a modest speed.
        goal = np.array(WAYPOINTS[self.waypoint], dtype=float)
        delta = goal - self.target
        if np.linalg.norm(delta) < 1e-3:
            self.waypoint = (self.waypoint + 1) % len(WAYPOINTS)
            goal = np.array(WAYPOINTS[self.waypoint], dtype=float)
            delta = goal - self.target
        direction = delta / (np.linalg.norm(delta) + 1e-9)
        speed = 0.55
        self.target += direction * speed * dt

        to_target = self.target - pos_xy
        self.distance = float(np.linalg.norm(to_target))
        if self.distance < 1e-3:
            return self.command

        desired = to_target / self.distance
        forward = np.array([math.cos(yaw), math.sin(yaw)])
        slow = 1.0
        for x, y, _half, _h in OBSTACLES:
            obs = np.array([x, y], dtype=float)
            away = pos_xy - obs
            dist = float(np.linalg.norm(away))
            if dist > 1.25:
                continue
            to_obs = obs - pos_xy
            if float(np.dot(to_obs / (np.linalg.norm(to_obs) + 1e-9), forward)) > 0.15:
                desired = desired + 1.6 * (1.25 - dist) * (away / (dist + 1e-9))
                slow = min(slow, 0.35 + 0.5 * dist / 1.25)
        desired /= np.linalg.norm(desired) + 1e-9
        heading = math.atan2(desired[1], desired[0])
        yaw_error = wrap(heading - yaw)
        standoff = 1.2
        if self.distance < standoff:
            vx = -0.5 * (standoff - self.distance)
        else:
            vx = min(0.85, 1.2 * (self.distance - standoff))
        vx *= slow
        wz = float(np.clip(1.8 * yaw_error, -0.9, 0.9))
        self.command = np.array([vx, 0.0, wz], dtype=np.float32)
        return self.command


def draw_hud(image, t, dist, command, stage):
    frame = Image.fromarray(image).convert("RGBA")
    overlay = Image.new("RGBA", frame.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    bold = ImageFont.truetype(FONT_BOLD, 32)
    mid = ImageFont.truetype(FONT_BOLD, 24)
    small = ImageFont.truetype(FONT_REG, 18)

    draw.rounded_rectangle((28, 22, 820, 116), radius=14, fill=(8, 12, 18, 195))
    draw.text((48, 32), "Unitree Go2 — target following + obstacle avoidance",
              font=bold, fill=(245, 245, 245, 255))
    draw.text((48, 76), f"AUTO mode · {stage} · privileged target/obstacle positions",
              font=small, fill=(170, 180, 190, 255))

    draw.rounded_rectangle((28, 136, 360, 194), radius=12, fill=(46, 130, 210, 220))
    draw.text((48, 146), "TARGET FOLLOW", font=mid, fill=(255, 255, 255, 255))
    draw.text((48, 172), f"distance {dist:4.2f} m", font=small, fill=(240, 240, 240, 230))

    draw.rounded_rectangle((28, 620, 1252, 700), radius=12, fill=(8, 12, 18, 180))
    draw.text((48, 632), f"command   vx {command[0]:+.2f} m/s    wz {command[2]:+.2f} rad/s",
              font=small, fill=(200, 210, 220, 255))
    draw.text((48, 660), f"t = {t:5.2f} s", font=small, fill=(140, 150, 160, 255))
    draw.text((700, 632), "policy: ONNX · balance by RL · steering by planner",
              font=small, fill=(160, 175, 190, 255))
    return np.array(Image.alpha_composite(frame, overlay).convert("RGB"))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--onnx", required=True)
    parser.add_argument("--out", default=os.path.join(ROOT, "doc", "demo", "go2-target-avoid.mp4"))
    parser.add_argument("--duration", type=float, default=18.0)
    parser.add_argument("--flat", action="store_true")
    parser.add_argument("--probe", action="store_true")
    parser.add_argument("--stage", default="")
    args = parser.parse_args()

    rough = not args.flat
    heightmap = "/tmp/go2_demo_terrain.png"
    make_heightmap(heightmap)
    scene = build_scene(rough, heightmap)
    model = mujoco.MjModel.from_xml_path(scene)
    model.opt.timestep = 0.005
    model.vis.global_.offwidth = WIDTH
    model.vis.global_.offheight = HEIGHT
    data = mujoco.MjData(model)
    runner = PolicyRunner(args.onnx, model, data)
    runner.reset()
    data.qpos[0:2] = [0.0, 0.0]
    data.qpos[2] = terrain_height(0.0, 0.0) + 0.32

    target_id = model.body("target").mocapid
    controller = HighLevel(WAYPOINTS[0])
    renderer = mujoco.Renderer(model, HEIGHT, WIDTH)
    camera = mujoco.MjvCamera()
    camera.elevation = -14.0

    control_every = int(round(0.02 / model.opt.timestep))
    step = 0
    next_render = 0.0
    frames = []
    probe_times = [1.0, 6.0, 12.0, 16.0]
    probe_index = 0
    try:
        while data.time < args.duration:
            if step % control_every == 0:
                pos = data.qpos[0:2].copy()
                yaw = yaw_from_quat(data.qpos[3:7])
                command = controller.update(pos, yaw, 0.02)
                runner.act(command)
                data.mocap_pos[target_id] = [controller.target[0], controller.target[1],
                                             terrain_height(*controller.target) + 0.30]
            runner.control()
            mujoco.mj_step(model, data)
            step += 1
            if data.time >= next_render:
                next_render += 1.0 / FPS
                camera.azimuth = 135.0
                camera.elevation = -33.0
                camera.distance = 4.0
                camera.lookat[0] = 0.0
                camera.lookat[1] = 0.0
                camera.lookat[2] = 0.25
                renderer.update_scene(data, camera=camera)
                stage = args.stage or ("ROUGH TERRAIN" if rough else "FLAT TERRAIN")
                image = draw_hud(renderer.render(), data.time, controller.distance,
                                 controller.command, stage)
                if args.probe:
                    if probe_index < len(probe_times) and data.time >= probe_times[probe_index]:
                        Image.fromarray(image).save(f"/tmp/go2_demo_probe_{probe_index}.png")
                        print("wrote", f"/tmp/go2_demo_probe_{probe_index}.png")
                        probe_index += 1
                else:
                    frames.append(image)
    finally:
        if not args.probe:
            os.makedirs(os.path.dirname(args.out), exist_ok=True)
            imageio.mimsave(args.out, frames, fps=FPS, quality=6, macro_block_size=None)
            print("wrote", args.out)


if __name__ == "__main__":
    main()
