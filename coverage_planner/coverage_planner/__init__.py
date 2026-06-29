"""coverage_planner package — drone search-area coverage planning (v3.0)."""

from coverage_planner.planner import (
    CoveragePlanner,
    gps_to_local,
    local_to_gps,
    max_safe_altitude,
    recall_vs_altitude_to_min_px,
    compute_coverage_pct,
    decompose_polygon_constrained,
    strip_aware_two_opt,
)

__all__ = [
    'CoveragePlanner',
    'gps_to_local',
    'local_to_gps',
    'max_safe_altitude',
    'recall_vs_altitude_to_min_px',
    'compute_coverage_pct',
    'decompose_polygon_constrained',
    'strip_aware_two_opt',
]
