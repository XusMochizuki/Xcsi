# /// script
# requires-python = ">=3.12"
# dependencies = ["numpy", "scipy", "pytest"]
# ///
"""Test 0 (no hardware): synthetic-signal tests of the IQ -> CSI chain.

    uv run research/repro/t0_selftest.py

Runs research/tests (802.11 HE SU packets with known multipath channel, CFO, fractional
delay, DC, 10-bit ADC, AWGN; fixed seeds). Use it to check a new machine / Python setup
before touching the bench.
"""
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.exit(pytest.main(["-q", os.path.join(ROOT, "research", "tests")]))
