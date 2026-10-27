#!/usr/bin/env python3
"""Preview scene: a target sphere orbits a center; cubes sit on the track.

This only builds the scene and renders stills -- no policy, no training.
"""
from __future__ import annotations

import math
import os

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE = os.path.join(ROOT, "src", "assets", "robots", "unitree_go2", "xmls", "scene_go2.xml")
MESH_DIR = os.path.join(ROOT, "src", "assets", "robots", "unitree_go2", "xmls", "assets")
OUT = os.path.join(ROOT, "doc", "demo")
FONT_BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
FONT_REG = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"

RADIUS = 2.2          # orbit radius [m]
SPEED = 0.60          # target linear speed [m/s]
OBSTACLE_ANGLES = [50.0, 135.0, 220.0, 305.0]  # degrees on the track
OBSTACLE_HALF = [0.18, 0.20, 0.17, 0.19]
OBSTACLE_HEIGHT = [0.50, 0.42, 0.55, 0.46]
BALL_Z = 0.90


def build_scene() -> str:
    with open(SCENE) as handle:
        xml = handle.read()
    xml = xml.replace('meshdir="assets"', f'meshdir="{MESH_DIR}"')

    extra = []
    # Dashed-looking orbit track, visual only.
    for i in range(96):
        angle = 2.0 * math.pi * i / 96.0
        x, y = RADIUS * math.cos(angle), RADIUS * math.sin(angle)
        extra.append(
            f'<geom name="track_{i}" type="cylinder" size="0.028 0.006" '
            f'pos="{x:.4f} {y:.4f} 0.006" rgba="0.10 0.62 0.95 1" '
            'contype="0" conaffinity="0"/>'
        )
    for index, (deg, half, height) in enumerate(
        zip(OBSTACLE_ANGLES, OBSTACLE_HALF, OBSTACLE_HEIGHT)
    ):
        angle = math.radians(deg)
        x, y = RADIUS * math.cos(angle), RADIUS * math.sin(angle)
        extra.append(
            f'<geom name="orbit_obstacle_{index}" type="box" '
            f'size="{half} {half} {height * 0.5}" '
            f'pos="{x:.4f} {y:.4f} {height * 0.5:.4f}" '
            'rgba="0.88 0.44 0.12 1"/>'
        )
    extra.append(
        f'<body name="orbit_target" mocap="true" pos="{RADIUS} 0 {BALL_Z}">'
        '<geom name="orbit_target_geom" type="sphere" size="0.15" '
        'rgba="1 0.10 0.08 1" contype="0" conaffinity="0"/></body>'
    )
    xml = xml.replace("</worldbody>", "  " + "\n  ".join(extra) + "\n  </worldbody>", 1)
    path = "/tmp/go2_orbit_scene.xml"
    with open(path, "w") as handle:
        handle.write(xml)
    return path


def hud(image, camera_name: str) -> np.ndarray:
    frame = Image.fromarray(image).convert("RGBA")
    overlay = Image.new("RGBA", frame.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    bold = ImageFont.truetype(FONT_BOLD, 34)
    mid = ImageFont.truetype(FONT_BOLD, 23)
    small = ImageFont.truetype(FONT_REG, 19)
    draw.rounded_rectangle((28, 22, 980, 128), radius=14, fill=(8, 12, 18, 205))
    draw.text((48, 32), "Go2 orbit-follow scene — preview", font=bold,
              fill=(245, 245, 245, 255))
    period = 2.0 * math.pi * RADIUS / SPEED
    draw.text((48, 80),
              f"orbit R = {RADIUS:.1f} m · target {SPEED:.2f} m/s "
              f"({period:.1f} s / lap) · {len(OBSTACLE_ANGLES)} cubes on track",
              font=small, fill=(185, 200, 215, 255))
    draw.text((48, 100), f"camera: {camera_name}", font=small,
              fill=(130, 145, 160, 255))
    draw.rounded_rectangle((28, 640, 520, 700), radius=12, fill=(8, 12, 18, 185))
    draw.text((48, 652), "robot start: center · target floats 0.9 m", font=mid,
              fill=(225, 235, 245, 255))
    return np.array(Image.alpha_composite(frame, overlay).convert("RGB"))


def render(model, data, camera, name: str) -> None:
    renderer = mujoco.Renderer(model, 720, 1280)
    try:
        renderer.update_scene(data, camera=camera)
        image = hud(renderer.render(), name)
    finally:
        renderer.close()
    path = os.path.join(OUT, f"orbit-scene-{name}.png")
    Image.fromarray(image).save(path)
    print("wrote", path)


def main() -> None:
    os.makedirs(OUT, exist_ok=True)
    model = mujoco.MjModel.from_xml_path(build_scene())
    model.vis.global_.offwidth = 1280
    model.vis.global_.offheight = 720
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    data.qpos[0:2] = [0.0, 0.0]
    data.qpos[2] = 0.32
    mujoco.mj_forward(model, data)

    top = mujoco.MjvCamera()
    top.lookat[:] = [0.0, 0.0, 0.25]
    top.distance = 6.2
    top.azimuth = 90.0
    top.elevation = -78.0

    persp = mujoco.MjvCamera()
    persp.lookat[:] = [0.0, 0.0, 0.45]
    persp.distance = 5.3
    persp.azimuth = 132.0
    persp.elevation = -24.0

    chase = mujoco.MjvCamera()
    chase.lookat[:] = [1.05, 0.0, 0.55]
    chase.distance = 3.0
    chase.azimuth = 180.0
    chase.elevation = -12.0

    render(model, data, top, "top")
    render(model, data, persp, "persp")
    render(model, data, chase, "chase")


if __name__ == "__main__":
    main()
