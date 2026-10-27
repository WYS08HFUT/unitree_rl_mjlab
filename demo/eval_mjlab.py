#!/usr/bin/env python3
"""Evaluate a checkpoint in the native mjlab env with a constant command."""
import argparse
import sys
from dataclasses import asdict

import torch


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--task", default="Unitree-Go2-Flat")
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--vx", type=float, default=0.6)
    parser.add_argument("--steps", type=int, default=200)
    args = parser.parse_args()
    sys.argv = [sys.argv[0]]
    import src.tasks  # noqa: F401
    from mjlab.envs import ManagerBasedRlEnv
    from mjlab.rl import RslRlVecEnvWrapper
    from mjlab.tasks.registry import load_env_cfg, load_rl_cfg, load_runner_cls

    cfg = load_env_cfg(args.task, play=True)
    cfg.scene.num_envs = 1
    cfg.commands["twist"].resampling_time_range = (1e9, 1e9)
    env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
    vec = RslRlVecEnvWrapper(env)
    runner = load_runner_cls(args.task)(vec, asdict(load_rl_cfg(args.task)),
                                        "logs/rsl_rl/tmp_eval", "cuda:0")
    runner.load(args.checkpoint, load_cfg={"actor": True}, strict=True)
    policy = runner.get_inference_policy(device="cuda:0")
    obs, _ = vec.reset()
    term = env.command_manager.get_term("twist")
    term.is_standing_env[:] = False
    term.command[:, 0] = args.vx
    term.command[:, 1] = 0.0
    term.command[:, 2] = 0.0
    start = env.scene["robot"].data.root_link_pos_w[0, :2].clone()
    with torch.no_grad():
        for _ in range(args.steps):
            obs, _, _, _ = vec.step(policy(obs))
            term.command[:, 0] = args.vx
            term.command[:, 1] = 0.0
            term.command[:, 2] = 0.0
    pos = env.scene["robot"].data.root_link_pos_w[0]
    dist = float(torch.linalg.norm(pos[:2] - start))
    print(f"native mjlab: distance={dist:.2f} m  z={float(pos[2]):.3f}")
    env.close()


if __name__ == "__main__":
    main()
