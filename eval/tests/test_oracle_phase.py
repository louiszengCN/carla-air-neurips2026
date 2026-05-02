"""Verify that the oracle landing-phase decoder implements paper Eq. (2)
exactly (App. C.7).

  phi* = approach   if d > 8 m
         descend    if 2 m < d <= 8 m  AND  cos(theta) >= 0.7
         hover      otherwise
"""
from dataclasses import dataclass
from typing import Tuple

from carlaair_eval.runtime.runner import _oracle_landing_phase
from carlaair_eval.api.phase_decoder import LandingPhase


@dataclass
class _State:
    uav_world: Tuple[float, float, float]
    bed_world: Tuple[float, float, float]
    uav_velocity_ned: Tuple[float, float, float]


def test_approach_when_d_gt_8m():
    s = _State(uav_world=(10.0, 0.0, 5.0),
               bed_world=(0.0, 0.0, 0.0),
               uav_velocity_ned=(0.0, 0.0, 0.0))
    # d = sqrt(100 + 25) = 11.18 > 8
    assert _oracle_landing_phase(s) == LandingPhase.APPROACH


def test_descend_when_d_in_band_and_velocity_vertical():
    # d = 5 (in 2..8); velocity straight down → cos(theta) = 1 >= 0.7
    s = _State(uav_world=(3.0, 0.0, 4.0),
               bed_world=(0.0, 0.0, 0.0),
               uav_velocity_ned=(0.0, 0.0, 1.0))
    assert _oracle_landing_phase(s) == LandingPhase.DESCEND


def test_hover_when_d_in_band_but_velocity_not_vertical():
    # d in band but velocity horizontal → cos(theta) = 0 < 0.7 → hover.
    s = _State(uav_world=(3.0, 0.0, 4.0),
               bed_world=(0.0, 0.0, 0.0),
               uav_velocity_ned=(1.0, 0.0, 0.0))
    assert _oracle_landing_phase(s) == LandingPhase.HOVER


def test_hover_when_d_le_2m():
    # d = 1.5 → falls through to hover branch.
    s = _State(uav_world=(0.5, 0.5, 1.0),
               bed_world=(0.0, 0.0, 0.0),
               uav_velocity_ned=(0.0, 0.0, 1.0))
    assert _oracle_landing_phase(s) == LandingPhase.HOVER


def test_descend_boundary_at_d_eq_8():
    # d == 8 falls in 2 < d <= 8 band.
    s = _State(uav_world=(8.0, 0.0, 0.0),
               bed_world=(0.0, 0.0, 0.0),
               uav_velocity_ned=(0.0, 0.0, 1.0))
    assert _oracle_landing_phase(s) == LandingPhase.DESCEND


def test_approach_just_above_8m():
    s = _State(uav_world=(8.001, 0.0, 0.0),
               bed_world=(0.0, 0.0, 0.0),
               uav_velocity_ned=(0.0, 0.0, 1.0))
    assert _oracle_landing_phase(s) == LandingPhase.APPROACH
