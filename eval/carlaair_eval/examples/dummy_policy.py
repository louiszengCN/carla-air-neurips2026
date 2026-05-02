"""Reference example showing how to plug a UAV policy into the eval suite.

This policy hovers in place and slowly descends — a deliberately weak
baseline so users can verify the full pipeline (scenario → cue → action
→ trace → metrics) end to end before plugging in a real model.
"""
from __future__ import annotations

import math
from typing import Optional

from ..api.policy import (
    UAVPolicy, UAVAction, VelocityCommand, Observation, PartnerCue,
)


class DummyHoverPolicy(UAVPolicy):
    def reset(self, task_instruction: str, task_name: str, episode_id: int) -> None:
        self._task = task_name

    def act(self, observation: Observation, partner_cue: Optional[PartnerCue]) -> UAVAction:
        # Tiny descend command so behaviour is non-static (good smoke test).
        return VelocityCommand(vx=0.0, vy=0.0, vz=0.5, yaw_deg=0.0, duration_s=0.25)
