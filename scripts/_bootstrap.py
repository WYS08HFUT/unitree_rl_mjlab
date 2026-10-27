"""Runtime bootstrap shared by the scripts in this directory.

Call both helpers *before* importing mjlab, warp or ``src.tasks``:

  from _bootstrap import add_repo_root_to_path, patch_warp_context_api

  add_repo_root_to_path()
  patch_warp_context_api()

Why this exists
---------------
1. ``add_repo_root_to_path`` makes ``import src`` work regardless of the
   working directory the script is launched from. Without it, running
   ``python scripts/train.py`` fails with ``ModuleNotFoundError: No module
   named 'src'`` unless the repo root happens to be on ``sys.path`` already.

2. ``patch_warp_context_api`` re-creates the ``warp.context`` namespace for
   warp-lang >= 1.13. mjlab 1.3.0 reads ``wp.context.runtime.driver_version``
   in ``mjlab/sim/sim.py`` and annotates ``wp.context.Device`` in
   ``mjlab/sensor/raycast_sensor.py``; warp 1.13 removed that namespace
   (upstream mjlab switched to ``warp.get_cuda_driver_version()`` in a later
   release). Without the shim, creating any ``ManagerBasedRlEnv`` dies with
   ``AttributeError: module 'warp' has no attribute 'context'``.
"""

from __future__ import annotations

import sys
import types
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent


def add_repo_root_to_path() -> None:
  """Put the repository root on ``sys.path`` so ``import src`` resolves."""
  root = str(REPO_ROOT)
  if root not in sys.path:
    sys.path.insert(0, root)


def patch_warp_context_api() -> None:
  """Restore ``warp.context`` (with ``.runtime.driver_version``) on warp >= 1.13."""
  import warp as wp

  if hasattr(wp, "context"):
    return

  context = types.ModuleType("warp.context")
  context.Device = wp.Device

  class _Runtime:
    """Minimal stand-in for warp's pre-1.13 CUDA runtime context."""

    @property
    def driver_version(self) -> tuple[int, int] | None:
      try:
        return wp.get_cuda_driver_version()
      except Exception:  # pragma: no cover - depends on driver availability
        return None

  context.runtime = _Runtime()
  sys.modules["warp.context"] = context
  wp.context = context
