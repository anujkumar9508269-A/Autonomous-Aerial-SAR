"""
tests/test_planner.py
=======================
Automated regression test suite for planner.py.

Run with: python3 tests/test_planner.py

Covers every scenario found during development and verification:
  - Square-FOV L-shape (original PS demo polygon) — all 3 strategies
  - Non-square-FOV large polygon — exposed FIX-13/FIX-14 boundary bug
  - 5-pointed star — stress test for non-convex decomposition
  - Narrow corridor — exposed FIX-10 (zero-strip fallback)
  - Tiny polygon — exposed FIX-11 (spiral loop guard)
  - Polygon with a hole — area-conservation check
  - Simple rectangle — baseline sanity check

Each test asserts coverage >= a documented threshold (95% for the PS's
end-evaluation target, except the star which is allowed 97% due to the
known spike-tip limitation — see README "Known limitations").
"""

import sys
import os
import math
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from shapely.geometry import Polygon

from planner import (
    CoveragePlanner,
    compute_coverage_pct,
    decompose_polygon_constrained,
)


PASS = 'PASS'
FAIL = 'FAIL'
results = []


def check(name, condition, detail=''):
    status = PASS if condition else FAIL
    results.append((name, status, detail))
    print(f'[{status}] {name}  {detail}')
    return condition


def star_polygon(n_points=5, r_outer=50, r_inner=20, cx=50, cy=50):
    pts = []
    for i in range(n_points * 2):
        r = r_outer if i % 2 == 0 else r_inner
        angle = math.pi * i / n_points
        pts.append((cx + r * math.sin(angle), cy + r * math.cos(angle)))
    return pts


def main():
    print('=' * 70)
    print('  planner.py regression suite')
    print('=' * 70)

    # ---- 1. Simple rectangle, square FOV --------------------------------
    planner = CoveragePlanner(altitude=20, hfov_deg=60, vfov_deg=60, overlap=0.1)
    rect = [(0, 0), (100, 0), (100, 80), (0, 80)]
    wps  = planner.generate_path(rect)
    fp   = planner.get_detection_footprints(wps)
    cov  = compute_coverage_pct(fp, rect, grid_step=1.0)
    check('Rectangle, square FOV, lawnmower', cov >= 99.0, f'coverage={cov:.1f}%')

    # ---- 2. L-shape demo polygon, all 3 strategies, square FOV -----------
    polygon_coords = [(0, 0), (100, 0), (100, 80), (60, 80), (60, 40), (0, 40)]
    for name, fn in [
        ('Lawnmower',  planner.generate_path),
        ('Non-convex', planner.generate_path_nonconvex),
        ('Spiral',     planner.generate_spiral_path),
    ]:
        wps = fn(polygon_coords)
        fp  = planner.get_detection_footprints(wps)
        cov = compute_coverage_pct(fp, polygon_coords, grid_step=1.0)
        check(f'L-shape demo polygon — {name}', cov >= 95.0,
              f'coverage={cov:.1f}% wps={len(wps)}')

    # ---- 3. Non-square FOV, larger polygon (FIX-13/14 regression) --------
    planner_ns = CoveragePlanner(altitude=30, hfov_deg=70, vfov_deg=55, overlap=0.1)
    big_poly = [(0, 0), (500, 0), (500, 400), (300, 400), (300, 200), (0, 200)]
    t0  = time.time()
    wps = planner_ns.generate_path_nonconvex(big_poly)
    fp  = planner_ns.get_detection_footprints(wps)
    cov = compute_coverage_pct(fp, big_poly, grid_step=2.0)
    elapsed = time.time() - t0
    check('Large polygon, non-square FOV (FIX-13/14)', cov >= 95.0,
          f'coverage={cov:.1f}% wps={len(wps)} plan+cov_time={elapsed:.2f}s')

    # ---- 4. Star polygon — non-convex decomposition stress test ----------
    planner_star = CoveragePlanner(altitude=15, hfov_deg=50, vfov_deg=50, overlap=0.15)
    star = star_polygon()
    subs = decompose_polygon_constrained(Polygon(star))
    area_diff = abs(sum(s.area for s in subs) - Polygon(star).area)
    check('Star polygon — decomposition area conserved', area_diff < 1e-6,
          f'area_diff={area_diff:.2e}')
    wps = planner_star.generate_path_nonconvex(star)
    fp  = planner_star.get_detection_footprints(wps)
    cov = compute_coverage_pct(fp, star, grid_step=1.0)
    # Known limitation: convex-hull merge approximation leaves small slivers
    # uncovered at sharp spike tips. 97% threshold documents this explicitly
    # rather than silently passing or failing on an unstated assumption.
    check('Star polygon — coverage (spike-tip limitation documented)',
          cov >= 97.0, f'coverage={cov:.1f}%')

    # ---- 5. Narrow corridor (FIX-10 regression) ---------------------------
    narrow = [(0, 0), (10, 0), (10, 200), (0, 200)]
    wps = planner.generate_path(narrow)
    fp  = planner.get_detection_footprints(wps)
    cov = compute_coverage_pct(fp, narrow, grid_step=1.0)
    check('Narrow corridor (FIX-10)', cov >= 95.0 and len(wps) > 0,
          f'coverage={cov:.1f}% wps={len(wps)}')

    # ---- 6. Tiny polygon (FIX-11 regression) -------------------------------
    tiny = [(0, 0), (15, 0), (15, 15), (0, 15)]
    wps_lawn   = planner.generate_path(tiny)
    wps_spiral = planner.generate_spiral_path(tiny)
    fp_lawn    = planner.get_detection_footprints(wps_lawn)
    fp_spiral  = planner.get_detection_footprints(wps_spiral)
    cov_lawn   = compute_coverage_pct(fp_lawn, tiny, grid_step=0.5)
    cov_spiral = compute_coverage_pct(fp_spiral, tiny, grid_step=0.5)
    check('Tiny polygon — lawnmower (FIX-11)', cov_lawn >= 95.0 and len(wps_lawn) > 0,
          f'coverage={cov_lawn:.1f}% wps={len(wps_lawn)}')
    check('Tiny polygon — spiral fallback (FIX-11)',
          cov_spiral >= 95.0 and len(wps_spiral) > 0,
          f'coverage={cov_spiral:.1f}% wps={len(wps_spiral)}')

    # ---- 7. Polygon with a hole — area conservation ------------------------
    exterior = [(0, 0), (100, 0), (100, 100), (0, 100)]
    hole     = [(40, 40), (60, 40), (60, 60), (40, 60)]
    poly_with_hole = Polygon(exterior, [hole])
    subs = decompose_polygon_constrained(poly_with_hole)
    area_diff = abs(sum(s.area for s in subs) - poly_with_hole.area)
    check('Polygon with hole — area conserved', area_diff < 1e-6,
          f'area_diff={area_diff:.2e} num_subs={len(subs)}')

    # ---- 8. GPS round-trip -------------------------------------------------
    from planner import gps_to_local, local_to_gps
    origin = (22.5195, 75.9278)
    gps_poly = local_to_gps(polygon_coords, *origin)
    roundtrip = gps_to_local(gps_poly, *origin)
    max_err = max(
        math.hypot(rx - lx, ry - ly)
        for (lx, ly), (rx, ry) in zip(polygon_coords, roundtrip)
    )
    check('GPS round-trip conversion', max_err < 0.01, f'max_err={max_err:.5f}m')

    # ---- 9. SAR-scale performance (FIX-15 regression) ---------------------
    planner_perf = CoveragePlanner(altitude=40, hfov_deg=65, vfov_deg=50, overlap=0.1)
    perf_poly = [(0, 0), (1000, 0), (1000, 800), (0, 800)]
    t0  = time.time()
    wps = planner_perf.generate_path(perf_poly)
    fp  = planner_perf.get_detection_footprints(wps)
    plan_time = time.time() - t0
    t0  = time.time()
    cov = compute_coverage_pct(fp, perf_poly, grid_step=2.0)
    cov_time = time.time() - t0
    # Threshold: vectorised coverage calc should comfortably finish under 1s
    # for a 1km x 800m area at 2m grid resolution (~200k sample points).
    # Pre-FIX-15 this took >3s; this asserts the regression stays fixed.
    check('SAR-scale performance (1km x 800m, FIX-15)',
          cov_time < 1.0 and cov >= 99.0,
          f'plan={plan_time:.3f}s cov_calc={cov_time:.3f}s coverage={cov:.1f}%')

    # ---- 10. Model-aware altitude calibration -----------------------------
    from planner import recall_vs_altitude_to_min_px, max_safe_altitude

    data_normal = [(20, 0.97), (40, 0.93), (60, 0.84), (80, 0.65), (100, 0.41)]
    r_normal = recall_vs_altitude_to_min_px(data_normal, min_acceptable_recall=0.85)
    check('Altitude calibration — normal crossing, no warning',
          r_normal['warning'] is None and 50 < r_normal['max_safe_altitude_m'] < 65,
          f"alt={r_normal['max_safe_altitude_m']:.1f}m min_px={r_normal['min_px']}")

    data_never_drops = [(20, 0.99), (40, 0.97), (60, 0.95)]
    r_never = recall_vs_altitude_to_min_px(data_never_drops, min_acceptable_recall=0.85)
    check('Altitude calibration — never drops below threshold (warns)',
          r_never['warning'] is not None,
          f"warning_present={r_never['warning'] is not None}")

    data_too_low = [(50, 0.70), (80, 0.50), (100, 0.30)]
    r_low = recall_vs_altitude_to_min_px(data_too_low, min_acceptable_recall=0.85)
    check('Altitude calibration — already below threshold at lowest alt (warns)',
          r_low['warning'] is not None,
          f"warning_present={r_low['warning'] is not None}")

    raised = False
    try:
        recall_vs_altitude_to_min_px([(20, 0.9)])
    except ValueError:
        raised = True
    check('Altitude calibration — rejects < 2 data points', raised)

    # Cross-check: feeding the derived min_px back into max_safe_altitude
    # must reproduce the same altitude (mutual consistency between the
    # two functions).
    reproduced = max_safe_altitude(min_px=r_normal['min_px'])
    check('Altitude calibration — round-trips through max_safe_altitude',
          abs(reproduced - r_normal['max_safe_altitude_m']) < 0.01,
          f"diff={abs(reproduced - r_normal['max_safe_altitude_m']):.4f}m")

    # ---- Summary -------------------------------------------------------------
    print('=' * 70)
    n_pass = sum(1 for _, s, _ in results if s == PASS)
    n_total = len(results)
    print(f'  {n_pass}/{n_total} checks passed')
    print('=' * 70)

    return n_pass == n_total


if __name__ == '__main__':
    ok = main()
    sys.exit(0 if ok else 1)
