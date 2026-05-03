"""Paper-strict constants — every value here is taken verbatim from the paper.

Reference: CarlaAir NeurIPS 2026 submission (round6).

Cross-references:
- Landing time limit / cargo-bed success:    §4.1, App. C.1
- C2 controller v0 / v_ref / clip range:     Eq. (1), App. C.2
- TSR window K:                              App. C.4 ("Tracking Success Rate")
- CCR floor ε:                               App. C.4 ("Cooperative Conversion Rate")
- Touch-down stability (≤0.3 m within 2 s): App. C.4 ("Landing Success Rate")
- Escort time limit / RSR threshold / RAT:  §4.1, App. C.4 ("Occlusion-recovery metrics")
- Occlusion event distribution:              App. C.4
- Statistical analysis:                      App. C.4 ("Statistical analysis")
"""

# ── Landing ───────────────────────────────────────────────────────────────
LANDING_EPISODE_SECONDS    = 60.0
LANDING_TOUCHDOWN_HOLD_S   = 2.0    # hold window after first contact
LANDING_TOUCHDOWN_DRIFT_M  = 0.3    # max drift inside hold window

# ── Escort ────────────────────────────────────────────────────────────────
ESCORT_EPISODE_SECONDS     = 90.0
RSR_IOU_THRESHOLD          = 0.15
RSR_HOLD_SECONDS           = 0.5
RAT_CAP_SECONDS            = 15.0
OCCLUSION_DURATION_RANGE   = (4.0, 12.0)   # uniform
OCCLUSIONS_PER_EPISODE     = (1, 3)        # inclusive
# Occlusion-geometry mix (App. C.4): minimal release implements "bridge" only.
# We document the full mix so reviewers can see the design intent.
OCCLUSION_GEOMETRY_MIX     = {"bridge": 0.40, "building": 0.35, "artifact": 0.25}
OCCLUSION_GEOMETRY_ENABLED = ("bridge",)

# ── TSR / Tracking primitive ──────────────────────────────────────────────
TSR_VISIBLE_SECONDS_K      = 3.0   # cumulative target-in-view time

# ── CCR ───────────────────────────────────────────────────────────────────
CCR_EPSILON                = 0.05

# ── C2 cooperative controller (paper Eq. 1, App. C.2) ────────────────────
#
#   v_UGV = v0 · clip( ‖v_UAV^fwd‖ / v_ref ,  clip_low,  clip_high )
#
# v0 is the nominal UGV speed. v_ref is a reference scaling constant for
# the UAV's commanded forward-velocity magnitude. The clip operator bounds
# the multiplicative factor to [0.5×, 1.5×] to prevent extreme values.
C2_V0_MS                   = 4.0
C2_V_REF_MS                = 2.0
C2_CLIP_LOW                = 0.5
C2_CLIP_HIGH               = 1.5

# ── Statistical protocol ──────────────────────────────────────────────────
DEFAULT_SEEDS              = 3
DEFAULT_EPISODES_PER_SEED  = 50
BOOTSTRAP_RESAMPLES        = 1000
BOOTSTRAP_CI_LEVEL         = 0.95
