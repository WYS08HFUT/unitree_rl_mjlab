"""Legacy installation entry point.

All packaging metadata lives in ``pyproject.toml``; this shim only keeps
``python setup.py ...`` invocations working for older tooling.
"""

from setuptools import setup

setup()
