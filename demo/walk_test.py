#!/usr/bin/env python3
"""Minimal ONNX policy runner for Go2 in MuJoCo (deploy.yaml contract).

Obs: base_ang_vel(3), projected_gravity(3), command(3), phase(2),
     joint_pos_rel(12), joint_vel_rel(12), last_action(12) = 47.
Action: 12 raw policy outputs -> q_target = default + 0.25 * action.
"""
from __future__ import annotations

import argparse
import math
import os

import mujoco
import numpy as np
import onnxruntime as ort

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE = os.path.join(ROOT, "src", "assets", "robots", "unitree_go2", "xmls", "scene_go2.xml")
STEP_DT = 0.02
GAIT_PERIOD = 0.6


class PolicyRunner:
    def __init__(self, onnx_path: str, model, data):
        self.session = ort.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])
        meta = self.session.get_modelmeta().custom_metadata_map
        self.joint_names = meta["joint_names"].split(",")
        self.default = np.array([float(x) for x in meta["default_joint_pos"].split(",")])
        self.kp = np.array([float(x) for x in meta["joint_stiffness"].split(",")])
        self.kd = np.array([float(x) for x in meta["joint_damping"].split(",")])
        self.scale = float(meta["action_scale"])
        self.model, self.data = model, data
        # map policy joint order -> xml joint qpos/dof and actuator index
        self.qadr, self.dadr, self.act_ids = [], [], []
        for name in self.joint_names:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            self.qadr.append(int(model.jnt_qposadr[jid]))
            self.dadr.append(int(model.jnt_dofadr[jid]))
            aid = next(i for i in range(model.nu) if int(model.actuator_trnid[i, 0]) == jid)
            self.act_ids.append(aid)
        self.qadr = np.array(self.qadr); self.dadr = np.array(self.dadr); self.act_ids = np.array(self.act_ids)
        self.lo = model.actuator_ctrlrange[self.act_ids, 0]
        self.hi = model.actuator_ctrlrange[self.act_ids, 1]
        self.phase = 0.0
        self.last_action = np.zeros(12, dtype=np.float32)
        self.obs = np.zeros((1, 47), dtype=np.float32)
        self.target = self.default.copy()

    def reset(self):
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        self.data.qpos[self.qadr] = self.default
        self.data.qpos[2] = 0.32
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        self.phase = 0.0
        self.last_action[:] = 0.0
        self.target = self.default.copy()
        for _ in range(100):  # let it settle on the default pose
            self.control()
            mujoco.mj_step(self.model, self.data)

    def quat(self):
        return self.data.qpos[3:7]

    def projected_gravity(self):
        w, x, y, z = self.quat()
        # q^-1 * (0,0,-1)
        return np.array([
            -2 * (x * z - w * y),
            -2 * (y * z + w * x),
            -(1 - 2 * (x * x + y * y)),
        ], dtype=np.float32)

    def control(self):
        """Position-servo torque, recomputed every physics step like mjlab."""
        q = self.data.qpos[self.qadr]
        dq = self.data.qvel[self.dadr]
        tau = self.kp * (self.target - q) - self.kd * dq
        self.data.ctrl[self.act_ids] = np.clip(tau, self.lo, self.hi)

    def act(self, command):
        gyro = self.data.sensor("imu_gyro").data.copy()
        q = self.data.qpos[self.qadr]
        dq = self.data.qvel[self.dadr]
        cmd = np.asarray(command, dtype=np.float32)
        norm = float(np.linalg.norm(cmd))
        self.phase = (self.phase + STEP_DT / GAIT_PERIOD) % 1.0
        phase = np.array([math.sin(2 * math.pi * self.phase), math.cos(2 * math.pi * self.phase)], dtype=np.float32)
        if norm < 0.1:
            phase[:] = 0.0
        obs = np.concatenate([
            gyro, self.projected_gravity(), cmd, phase,
            q - self.default, dq, self.last_action,
        ]).astype(np.float32)[None, :]
        action = self.session.run(None, {self.session.get_inputs()[0].name: obs})[0][0].astype(np.float32)
        self.last_action = action
        self.target = self.default + self.scale * action
        return action


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--onnx", required=True)
    parser.add_argument("--vx", type=float, default=0.6)
    parser.add_argument("--seconds", type=float, default=6.0)
    parser.add_argument("--out", default=None)
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(SCENE)
    model.opt.timestep = 0.005
    model.vis.global_.offwidth = 1280
    model.vis.global_.offheight = 720
    data = mujoco.MjData(model)
    runner = PolicyRunner(args.onnx, model, data)
    runner.reset()
    start = data.qpos[:2].copy()
    command = np.array([args.vx, 0.0, 0.0], dtype=np.float32)
    control_every = int(round(STEP_DT / model.opt.timestep))
    frames = []
    renderer = None
    if args.out:
        renderer = mujoco.Renderer(model, 720, 1280)
        cam = mujoco.MjvCamera(); cam.distance = 2.0; cam.azimuth = 130; cam.elevation = -18
    step = 0
    try:
        while data.time < args.seconds:
            if step % control_every == 0:
                runner.act(command)
            runner.control()
            mujoco.mj_step(model, data)
            step += 1
            if renderer is not None and step % 2 == 0:
                cam.lookat[0] = data.qpos[0]; cam.lookat[1] = data.qpos[1]; cam.lookat[2] = 0.3
                renderer.update_scene(data, camera=cam)
                frames.append(renderer.render())
    finally:
        if renderer is not None:
            renderer.close()
    dist = float(np.linalg.norm(data.qpos[:2] - start))
    print(f"final z={data.qpos[2]:.3f}  xy=({data.qpos[0]:.2f},{data.qpos[1]:.2f})  distance={dist:.2f} m")
    if args.out and frames:
        import imageio.v2 as imageio
        imageio.mimsave(args.out, frames, fps=30)
        print("wrote", args.out)


if __name__ == "__main__":
    main()
