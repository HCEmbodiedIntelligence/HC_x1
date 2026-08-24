#!/usr/bin/env python3
"""Compatibility entry point for the renamed low-latency control adapter."""

from __future__ import annotations

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from hc_x1.control_adapter import HcX1ControlAdapter, main

HcX1StandardAdapter = HcX1ControlAdapter

__all__ = ["HcX1ControlAdapter", "HcX1StandardAdapter", "main"]


if __name__ == "__main__":
    main()
