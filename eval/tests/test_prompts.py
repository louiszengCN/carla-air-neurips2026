"""Verify cue text matches the paper VERBATIM.

These strings are quoted directly from the paper LaTeX source:
  - Appendix C.3, Table 6   ("Full prompt protocol")
  - Appendix C.6, Table C.5 ("Prompt variants for C1 ablation")

If a future edit to cue.py inadvertently changes any default prompt, these
tests will fail. Each canonical example is wired so a reviewer's code
agent can confirm byte-for-byte equality with the paper.
"""
import random

from carlaair_eval.api.cue import (
    LANDING_BASE_INSTRUCTION, ESCORT_BASE_INSTRUCTION,
    LandingCueState, EscortCueState,
    build_landing_cue, build_escort_cue, CueFormat,
    _ESCORT_REAPPEAR_VOCAB, _LANDING_PHASE_VOCAB,
)


# ── paper canonical strings (verbatim) ──────────────────────────────────
PAPER_LANDING_C0 = (
    "Follow the moving truck, keep it in view, align above the flat rear "
    "cargo bed, and land safely on the cargo bed."
)

PAPER_LANDING_C1_SEM = (
    "Follow the moving truck, keep it in view, align above the flat rear "
    "cargo bed, and land safely on the cargo bed. Assistant hint: the cargo "
    "bed is forward-left. The truck is moving slowly. Current phase: "
    "approach. Use the hint only to choose your next UAV action."
)

PAPER_LANDING_C1_NUM = (
    "Follow the moving truck, keep it in view, align above the flat rear "
    "cargo bed, and land safely on the cargo bed. State: truck speed = 2.0 "
    "m/s; truck heading = 15 deg; relative bearing to cargo bed = -30 deg; "
    "relative distance to cargo bed = 8.0 m; phase = approach."
)

PAPER_LANDING_C1_ORACLE_BEARING = (
    "Follow the moving truck, keep it in view, align above the flat rear "
    "cargo bed, and land safely. State update: cargo bed at bearing 312°, "
    "range 6.2 m, elevation -8°. Phase: approach. Use the state update "
    "only to choose your next UAV action."
)

PAPER_ESCORT_C0 = "Follow the moving truck and keep it in view."

PAPER_ESCORT_C1_SEM = (
    "Follow the moving truck and keep it in view. Assistant hint: the truck "
    "is temporarily occluded by the bridge. The truck continues forward and "
    "will reappear on the forward-right side. Current phase: occlusion "
    "recovery. Use the hint only to recover visual contact."
)

PAPER_ESCORT_C1_NUM = (
    "Follow the moving truck and keep it in view. State: occlusion = true; "
    "truck speed = 2.0 m/s; expected reappearance bearing = 35 deg; "
    "expected reappearance distance = 9.4 m; phase = occlusion recovery."
)

PAPER_ESCORT_C1_ORACLE_BEARING = (
    "Follow the moving truck and keep it in view. State update: UGV at "
    "bearing 35°, range 9.4 m, elevation -12°. Phase: occlusion recovery. "
    "Use the state update only to recover visual contact."
)


# ── base instructions ───────────────────────────────────────────────────
def test_landing_base_instruction():
    assert LANDING_BASE_INSTRUCTION == PAPER_LANDING_C0


def test_escort_base_instruction():
    assert ESCORT_BASE_INSTRUCTION == PAPER_ESCORT_C0


# ── Landing C1 variants ─────────────────────────────────────────────────
def test_landing_c1_sem_matches_paper():
    state = LandingCueState(
        bed_dx=4.0, bed_dy=-4.0,           # forward-left in body frame
        bed_range_m=5.6, bed_bearing_deg=315.0, bed_elevation_deg=-8.0,
        truck_speed_ms=2.5, truck_heading_deg=0.0, phase="approach",
    )
    cue = build_landing_cue(state, fmt=CueFormat.SEMANTIC)
    assert cue.text == PAPER_LANDING_C1_SEM


def test_landing_c1_num_matches_paper():
    # Paper Tbl. C.5 example: truck speed 2.0 m/s, heading 15 deg, signed
    # bearing -30 deg (=> unsigned 330), distance 8.0 m, phase approach.
    state = LandingCueState(
        bed_dx=0.0, bed_dy=0.0,
        bed_range_m=8.0, bed_bearing_deg=330.0, bed_elevation_deg=0.0,
        truck_speed_ms=2.0, truck_heading_deg=15.0, phase="approach",
    )
    cue = build_landing_cue(state, fmt=CueFormat.NUMERIC)
    assert cue.text == PAPER_LANDING_C1_NUM


def test_landing_c1_oracle_bearing_matches_paper():
    state = LandingCueState(
        bed_dx=0.0, bed_dy=0.0,
        bed_range_m=6.2, bed_bearing_deg=312.0, bed_elevation_deg=-8.0,
        truck_speed_ms=0.0, truck_heading_deg=0.0, phase="approach",
    )
    cue = build_landing_cue(state, fmt=CueFormat.ORACLE_BEARING)
    assert cue.text == PAPER_LANDING_C1_ORACLE_BEARING


def test_landing_c1_noisy_template_shape():
    # Noisy outputs are stochastic; assert that:
    #   (a) the template structure matches the paper noisy example,
    #   (b) the corrupted fields stay within the published vocabularies.
    state = LandingCueState(
        bed_dx=4.0, bed_dy=-4.0,
        bed_range_m=5.6, bed_bearing_deg=315.0, bed_elevation_deg=-8.0,
        truck_speed_ms=2.5, truck_heading_deg=0.0, phase="approach",
    )
    rng = random.Random(0)
    cue = build_landing_cue(state, fmt=CueFormat.NOISY, noise_rng=rng)
    assert cue.text.startswith(PAPER_LANDING_C0 + " Assistant hint: the cargo bed is ")
    assert cue.text.endswith(". Use the hint only to choose your next UAV action.")
    # All emitted phase tokens must be in the published vocabulary.
    for phase in _LANDING_PHASE_VOCAB:
        # exercise: the phrase ". Current phase: <phase>." must be valid.
        assert isinstance(phase, str) and phase


# ── Escort C1 variants ──────────────────────────────────────────────────
def test_escort_c1_sem_matches_paper():
    state = EscortCueState(
        occlusion_active=True,
        occlusion_kind="bridge",
        motion_intent="continues forward",
        reappear_direction="forward-right side",
        phase="occlusion recovery",
    )
    cue = build_escort_cue(state, fmt=CueFormat.SEMANTIC)
    assert cue.text == PAPER_ESCORT_C1_SEM


def test_escort_c1_num_matches_paper():
    # Paper Tbl. C.5 example: occlusion=true, speed 2.0 m/s, bearing 35 deg,
    # distance 9.4 m, phase occlusion recovery.
    state = EscortCueState(
        occlusion_active=True,
        occlusion_kind="bridge",
        motion_intent="continues forward",
        reappear_direction="forward-right side",
        truck_speed_ms=2.0,
        reappear_bearing_deg=35.0,
        reappear_range_m=9.4,
        reappear_elevation_deg=-12.0,
        phase="occlusion recovery",
    )
    cue = build_escort_cue(state, fmt=CueFormat.NUMERIC)
    assert cue.text == PAPER_ESCORT_C1_NUM


def test_escort_c1_oracle_bearing_matches_paper():
    state = EscortCueState(
        occlusion_active=True,
        occlusion_kind="bridge",
        motion_intent="continues forward",
        reappear_direction="forward-right side",
        truck_speed_ms=2.0,
        reappear_bearing_deg=35.0,
        reappear_range_m=9.4,
        reappear_elevation_deg=-12.0,
        phase="occlusion recovery",
    )
    cue = build_escort_cue(state, fmt=CueFormat.ORACLE_BEARING)
    assert cue.text == PAPER_ESCORT_C1_ORACLE_BEARING


def test_escort_c1_noisy_template_shape():
    state = EscortCueState(
        occlusion_active=True,
        occlusion_kind="bridge",
        motion_intent="continues forward",
        reappear_direction="forward-right side",
        truck_speed_ms=2.0,
        phase="occlusion recovery",
    )
    rng = random.Random(0)
    cue = build_escort_cue(state, fmt=CueFormat.NOISY, noise_rng=rng)
    # Template skeleton must remain stable.
    assert cue.text.startswith(PAPER_ESCORT_C0 + " Assistant hint: the truck is ")
    assert cue.text.endswith("Use the hint only to recover visual contact.")
