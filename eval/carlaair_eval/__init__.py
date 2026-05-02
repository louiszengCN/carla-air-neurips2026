"""carlaair_eval — minimal eval suite for the CarlaAir cooperative VLA paper.

This package implements the diagnostic evaluation suite described in
Section 4 and Appendix C of the CarlaAir NeurIPS 2026 submission. It
provides scenarios, cooperation modes, prompt builders, metric computation,
and a state-based cooperative reference (Rule-Coop-State). It does NOT
provide UAV policies — users plug in their own policy via the
`UAVPolicy` interface.

Paper-strict constants are centralised in `carlaair_eval.constants`.
"""

__version__ = "0.1.0"
