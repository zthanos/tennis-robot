#!/usr/bin/env python3
"""P2-81 conservative control-panel rib screen (mm, N, MPa)."""
from __future__ import annotations

import json
import math
from pathlib import Path

E_PETG = 1500.0
LOAD_N = 30.0
SPAN_MM = 116.0
EFFECTIVE_TOP_WIDTH_MM = 40.0
TOP_T_MM = 4.0
RIB_TOTAL_WIDTH_MM = 16.0
RIB_DEPTH_MM = 8.0
POST_LENGTH_MM = 198.0
POST_DIAMETER_MM = 23.0


def main() -> None:
    top_area = EFFECTIVE_TOP_WIDTH_MM * TOP_T_MM
    rib_area = RIB_TOTAL_WIDTH_MM * RIB_DEPTH_MM
    top_centroid = TOP_T_MM / 2
    rib_centroid = -RIB_DEPTH_MM / 2
    neutral_axis = (
        top_area * top_centroid + rib_area * rib_centroid
    ) / (top_area + rib_area)
    inertia = (
        EFFECTIVE_TOP_WIDTH_MM * TOP_T_MM**3 / 12
        + top_area * (top_centroid - neutral_axis) ** 2
        + RIB_TOTAL_WIDTH_MM * RIB_DEPTH_MM**3 / 12
        + rib_area * (rib_centroid - neutral_axis) ** 2
    )
    panel_deflection = LOAD_N * SPAN_MM**3 / (48 * E_PETG * inertia)
    post_area = math.pi * POST_DIAMETER_MM**2 / 4
    post_shortening = LOAD_N * POST_LENGTH_MM / (4 * E_PETG * post_area)
    moment = LOAD_N * SPAN_MM / 4
    result = {
        "status": "P2_81_CONTROL_PANEL_RIB_SCREEN_PASS",
        "assumptions": {
            "material": "PETG linear isotropic screen",
            "elastic_modulus_MPa": E_PETG,
            "load_N": LOAD_N,
            "support_span_mm": SPAN_MM,
            "effective_top_width_mm": EFFECTIVE_TOP_WIDTH_MM,
            "credited_rib_width_mm": RIB_TOTAL_WIDTH_MM,
            "total_section_depth_mm": TOP_T_MM + RIB_DEPTH_MM,
            "joint_slip_credited": False,
        },
        "section": {
            "neutral_axis_mm_from_panel_underside_positive_up": neutral_axis,
            "second_moment_mm4": inertia,
        },
        "results": {
            "panel_midspan_deflection_mm": panel_deflection,
            "four_post_axial_shortening_mm": post_shortening,
            "combined_top_displacement_mm": panel_deflection + post_shortening,
            "top_fibre_bending_stress_MPa": moment * (TOP_T_MM - neutral_axis) / inertia,
            "bottom_fibre_bending_stress_MPa": moment * abs(-RIB_DEPTH_MM - neutral_axis) / inertia,
            "limit_mm": 0.3,
            "margin_to_limit_mm": 0.3 - panel_deflection - post_shortening,
        },
        "limitations": [
            "The two 8 mm E-stop rails are credited as 16 mm total rib width; the split transverse ties and wider grid are not credited.",
            "A physical 30 N proof-load remains required for print anisotropy, local yielding, fastener seating and creep.",
        ],
    }
    out = Path(__file__).resolve().parent / "results/shared_flywheel_phase2/p281_control_panel.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
