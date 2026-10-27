#!/usr/bin/env python3
"""Prepend a title card and concatenate demo segments into one mp4."""
from __future__ import annotations

import argparse
import os

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FONT_BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
FONT_REG = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"


def title_card(seconds: float = 2.5, fps: int = 30):
    width, height = 1280, 720
    image = Image.new("RGB", (width, height), (12, 18, 26))
    draw = ImageDraw.Draw(image)
    for y in range(height):
        shade = int(12 + 18 * (y / height))
        draw.line([(0, y), (width, y)], fill=(shade, shade + 6, shade + 14))
    title = ImageFont.truetype(FONT_BOLD, 52)
    sub = ImageFont.truetype(FONT_BOLD, 26)
    small = ImageFont.truetype(FONT_REG, 20)
    draw.text((90, 170), "Unitree Go2 — from flat to rough", font=title,
              fill=(240, 244, 248))
    draw.text((90, 250), "Learned locomotion · target following · obstacle avoidance",
              font=sub, fill=(120, 200, 255))
    draw.text((90, 330), "Stage 1   flat-terrain walking", font=small,
              fill=(200, 210, 220))
    draw.text((90, 366), "Stage 2   rough terrain + moving target + obstacle avoidance",
              font=small, fill=(200, 210, 220))
    draw.text((90, 440), "PPO policy trained in mjlab · 50 Hz ONNX inference · MuJoCo 3.5",
              font=small, fill=(150, 165, 180))
    draw.text((90, 620), "The planner uses privileged target/obstacle positions; balance is learned.",
              font=small, fill=(110, 120, 132))
    frame = np.array(image)
    return [frame.copy() for _ in range(int(seconds * fps))]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--segments", nargs="+", required=True)
    parser.add_argument("--out", default=os.path.join(ROOT, "doc", "demo",
                                                      "go2-target-avoidance.mp4"))
    args = parser.parse_args()

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    writer = imageio.get_writer(args.out, fps=30, codec="libx264", quality=8,
                                pixelformat="yuv420p", macro_block_size=None)
    try:
        for frame in title_card():
            writer.append_data(frame)
        for path in args.segments:
            reader = imageio.get_reader(path)
            for frame in reader:
                writer.append_data(frame)
            reader.close()
    finally:
        writer.close()
    print("wrote", args.out)


if __name__ == "__main__":
    main()
