"""
check_efficiency.py — standalone coverage planning efficiency checker
======================================================================
Run this BEFORE integrating with human detection or geo-tagging to verify
the coverage planner is working correctly and efficiently for YOUR mission
parameters (your polygon, your camera, your altitude).

No ROS, Gazebo, MAVROS, or detection model needed.

Usage:
    python3 check_efficiency.py                        # uses demo L-shape
    python3 check_efficiency.py --yaml mission.yaml    # your actual polygon
    python3 check_efficiency.py --gps                  # GPS polygon demo

What it checks and why each matters:
  1. Coverage completeness  — does the planned path actually cover the area?
                              (Failed silently in original code at only 33%)
  2. Coverage efficiency    — how much of the flight is useful vs transit?
  3. Strip validity         — are all waypoints inside the polygon?
  4. Altitude vs detection  — does the altitude make sense geometrically?
  5. Strategy comparison    — which strategy is best for your polygon shape?
  6. Scale timing           — how long will this mission take to fly?
"""

import sys
import os
import math
import time
import argparse
import yaml
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.patheffects as pe
from matplotlib.patches import Polygon as MplPolygon
from matplotlib.collections import PatchCollection
from shapely.geometry import Polygon, Point
from shapely.ops import unary_union

# Allow running from repo root or tests/ directory
sys.path.insert(0, os.path.dirname(__file__))
from planner import (
    CoveragePlanner,
    compute_coverage_pct,
    gps_to_local,
    local_to_gps,
    max_safe_altitude,
    recall_vs_altitude_to_min_px,
    _path_length,
    _dist,
)


# ---------------------------------------------------------------------------
# Configuration — edit these for your actual mission
# ---------------------------------------------------------------------------

# Your camera parameters
ALTITUDE_M   = 20.0    # planned flight altitude (m)
HFOV_DEG     = 60.0    # horizontal field of view (degrees)
VFOV_DEG     = 60.0    # vertical field of view (degrees)
OVERLAP      = 0.10    # strip overlap fraction (0.10 = 10%)

# Your drone's typical cruise speed (m/s) — used for flight time estimate
DRONE_SPEED_MS = 5.0

# Grid resolution for coverage sampling (metres). Smaller = more accurate
# but slower. 1.0 m is fine for most SAR areas up to ~500x500m.
# Use 2.0–5.0 for large areas (>1km).
GRID_STEP_M = 1.0

# Demo polygon (L-shape) — replace with your actual polygon or use --yaml
DEMO_POLYGON = [
    (0,   0),
    (100, 0),
    (100, 80),
    (60,  80),
    (60,  40),
    (0,   40),
]

# Optional: your measured recall-vs-altitude data for model-aware ceiling.
# Leave as None to skip model-aware check and use the geometric formula only.
# Format: list of (altitude_m, recall) tuples from your validation runs.
# Example: RECALL_DATA = [(20, 0.97), (40, 0.93), (60, 0.84), (80, 0.65)]
RECALL_DATA = None
MIN_ACCEPTABLE_RECALL = 0.85   # PS end-evaluation target


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

PASS = '✓ PASS'
WARN = '⚠ WARN'
FAIL = '✗ FAIL'


def check(label, status, detail=''):
    symbol = {'pass': PASS, 'warn': WARN, 'fail': FAIL}[status]
    print(f'  {symbol}  {label}')
    if detail:
        print(f'         {detail}')
    return status == 'pass'


def flight_time_estimate(waypoints, speed_ms):
    """Estimate total flight time from path length and cruise speed."""
    dist_m = _path_length(waypoints)
    return dist_m / speed_ms


def transit_fraction(waypoints, planner):
    """
    Fraction of total flight path spent on inter-strip transitions
    (not flying actual coverage strips). Lower is more efficient.
    Strips are segments where |Δy| < safe_width/2 in the sweep frame
    (i.e. moving along a strip, not turning between strips).
    Approximation: any segment shorter than safe_width/4 is a transition.
    """
    if len(waypoints) < 2:
        return 0.0
    total = _path_length(waypoints)
    transit = 0.0
    for i in range(len(waypoints) - 1):
        seg = _dist(waypoints[i], waypoints[i + 1])
        # Heuristic: segments much shorter than a strip are transitions
        if seg < planner.safe_width * 0.5:
            transit += seg
    return transit / total if total > 0 else 0.0


def waypoints_inside_polygon(waypoints, polygon_coords):
    """Return list of waypoints that fall OUTSIDE the search polygon."""
    poly = Polygon(polygon_coords).buffer(0.5)  # 0.5m tolerance
    outside = []
    for wp in waypoints:
        if not poly.contains(Point(wp[0], wp[1])):
            outside.append(wp)
    return outside


def load_yaml_polygon(yaml_path):
    """Load polygon from a YAML file (same format as coverage_planner_node)."""
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    if 'gps_polygon' in data:
        origin_lat = data['origin_lat']
        origin_lon = data['origin_lon']
        gps_poly = [tuple(p) for p in data['gps_polygon']]
        return gps_to_local(gps_poly, origin_lat, origin_lon), \
               (origin_lat, origin_lon), True
    elif 'polygon' in data:
        return [tuple(p) for p in data['polygon']], None, False
    else:
        raise ValueError('YAML must have "polygon" or "gps_polygon" key')


# ---------------------------------------------------------------------------
# Main efficiency report
# ---------------------------------------------------------------------------

def run_report(polygon_coords, gps_origin=None, title='Coverage Efficiency Report'):
    planner = CoveragePlanner(
        altitude=ALTITUDE_M,
        hfov_deg=HFOV_DEG,
        vfov_deg=VFOV_DEG,
        overlap=OVERLAP,
    )
    poly = Polygon(polygon_coords)

    print()
    print('=' * 68)
    print(f'  {title}')
    print('=' * 68)
    print(f'  Altitude       : {ALTITUDE_M} m')
    print(f'  HFOV / VFOV    : {HFOV_DEG}° / {VFOV_DEG}°')
    print(f'  Overlap        : {int(OVERLAP*100)}%')
    print(f'  Strip width    : {planner.strip_width:.2f} m')
    print(f'  Strip height   : {planner.strip_height:.2f} m')
    print(f'  Safe width     : {planner.safe_width:.2f} m')
    print(f'  Search area    : {poly.area:.0f} m²  '
          f'({poly.area/10000:.2f} ha)')
    print(f'  Drone speed    : {DRONE_SPEED_MS} m/s')
    print()

    # ----------------------------------------------------------------
    # Section 1: Altitude checks
    # ----------------------------------------------------------------
    print('─' * 68)
    print('  [1] Altitude vs Detection Ceiling')
    print('─' * 68)

    geom_ceiling = max_safe_altitude(vfov_deg=VFOV_DEG, min_px=20)
    margin = (geom_ceiling - ALTITUDE_M) / geom_ceiling * 100
    alt_ok = ALTITUDE_M <= geom_ceiling
    check('Altitude below geometric ceiling (min_px=20, generic)',
          'pass' if alt_ok else 'warn',
          f'ceiling={geom_ceiling:.1f}m  configured={ALTITUDE_M}m  '
          f'margin={margin:.0f}%')

    if RECALL_DATA:
        result = recall_vs_altitude_to_min_px(
            RECALL_DATA, min_acceptable_recall=MIN_ACCEPTABLE_RECALL
        )
        model_ceiling = result['max_safe_altitude_m']
        model_ok = ALTITUDE_M <= model_ceiling
        check('Altitude below MODEL-DERIVED ceiling (your YOLOv8 data)',
              'pass' if model_ok else 'fail',
              f'model_ceiling={model_ceiling:.1f}m  '
              f'configured={ALTITUDE_M}m  '
              f'min_px={result["min_px"]}  '
              f'{"WARNING: " + result["warning"] if result["warning"] else ""}')
    else:
        print(f'  ○ SKIP  Model-aware ceiling check (set RECALL_DATA to enable)')
        print(f'         Run your YOLOv8 at several altitudes, record recall,')
        print(f'         then add to RECALL_DATA at the top of this script.')
    print()

    # ----------------------------------------------------------------
    # Section 2: Strategy comparison
    # ----------------------------------------------------------------
    print('─' * 68)
    print('  [2] Strategy Comparison')
    print('─' * 68)

    strategies = {}
    for name, fn in [
        ('Lawnmower',  planner.generate_path),
        ('Non-convex', planner.generate_path_nonconvex),
        ('Spiral',     planner.generate_spiral_path),
    ]:
        t0  = time.time()
        wps = fn(polygon_coords)
        fp  = planner.get_detection_footprints(wps)
        cov = compute_coverage_pct(fp, polygon_coords, grid_step=GRID_STEP_M)
        t1  = time.time()
        dist_m   = _path_length(wps)
        fly_s    = flight_time_estimate(wps, DRONE_SPEED_MS)
        transit  = transit_fraction(wps, planner)
        outside  = waypoints_inside_polygon(wps, polygon_coords)
        strategies[name] = {
            'wps': wps, 'fp': fp, 'cov': cov,
            'dist_m': dist_m, 'fly_s': fly_s,
            'transit_pct': transit * 100,
            'outside': outside,
            'plan_time_ms': (t1 - t0) * 1000,
        }

    baseline_dist = strategies['Lawnmower']['dist_m']
    print(f'  {"Strategy":<14} {"WPs":>4} {"Coverage":>9} {"Dist(m)":>9} '
          f'{"vs base":>8} {"FlyTime":>8} {"Transit%":>9} {"OOB":>5}')
    print(f'  {"-"*14} {"-"*4} {"-"*9} {"-"*9} '
          f'{"-"*8} {"-"*8} {"-"*9} {"-"*5}')

    all_pass = True
    for name, s in strategies.items():
        t_str = f'{s["fly_s"]:.0f}s' if s["fly_s"] < 60 else \
                f'{s["fly_s"]/60:.1f}min'
        cov_ok  = s['cov'] >= 95.0
        oob_ok  = len(s['outside']) == 0
        flag = '✓' if (cov_ok and oob_ok) else '✗'
        print(f'  {flag} {name:<13} {len(s["wps"]):>4} '
              f'{s["cov"]:>8.1f}% {s["dist_m"]:>9.0f} '
              f'{s["dist_m"]/baseline_dist:>7.2f}x {t_str:>8} '
              f'{s["transit_pct"]:>8.1f}% {len(s["outside"]):>5}')
        if not (cov_ok and oob_ok):
            all_pass = False

    if not all_pass:
        print()
        for name, s in strategies.items():
            if s['cov'] < 95.0:
                print(f'  ✗ {name}: coverage {s["cov"]:.1f}% < 95% threshold')
            if s['outside']:
                print(f'  ✗ {name}: {len(s["outside"])} waypoints outside polygon:')
                for wp in s['outside'][:5]:
                    print(f'         {wp}')
    print()

    # ----------------------------------------------------------------
    # Section 3: Coverage detail for recommended strategy
    # ----------------------------------------------------------------
    best_name = max(strategies, key=lambda n: strategies[n]['cov'])
    best = strategies[best_name]

    print('─' * 68)
    print(f'  [3] Coverage Detail — Best Strategy: {best_name}')
    print('─' * 68)

    check('Coverage >= 95% (PS mid-evaluation target)',
          'pass' if best['cov'] >= 95.0 else 'fail',
          f'{best["cov"]:.2f}%')
    check('Coverage >= 99% (PS end-evaluation target)',
          'pass' if best['cov'] >= 99.0 else 'warn',
          f'{best["cov"]:.2f}%  '
          f'{"consider reducing overlap or altitude" if best["cov"] < 99.0 else ""}')
    check('All waypoints inside search polygon',
          'pass' if len(best['outside']) == 0 else 'fail',
          f'{len(best["outside"])} out-of-bounds waypoints')
    check('Planning time under 1s (real-time capable)',
          'pass' if best['plan_time_ms'] < 1000 else 'warn',
          f'{best["plan_time_ms"]:.1f} ms')
    print()

    # ----------------------------------------------------------------
    # Section 4: Scale projection
    # ----------------------------------------------------------------
    print('─' * 68)
    print(f'  [4] Mission Scale Estimate')
    print('─' * 68)

    area_m2    = poly.area
    strips_est = math.ceil(
        math.sqrt(area_m2) / planner.safe_width
    )
    wps_est    = strips_est * 2
    dist_est   = best['dist_m']
    time_est   = dist_est / DRONE_SPEED_MS

    print(f'  Estimated strips         : ~{strips_est}')
    print(f'  Actual waypoints planned : {len(best["wps"])}')
    print(f'  Total path distance      : {dist_est:.0f} m  '
          f'({dist_est/1000:.2f} km)')
    if time_est < 60:
        print(f'  Estimated flight time    : {time_est:.0f} s')
    elif time_est < 3600:
        print(f'  Estimated flight time    : {time_est/60:.1f} min')
    else:
        print(f'  Estimated flight time    : {time_est/3600:.1f} hr  '
              f'⚠ likely needs multiple batteries')
    print(f'  Area per waypoint        : {area_m2/max(len(best["wps"]),1):.0f} m²')
    print()

    # ----------------------------------------------------------------
    # Section 5: GPS round-trip (if GPS origin provided)
    # ----------------------------------------------------------------
    if gps_origin:
        print('─' * 68)
        print('  [5] GPS Conversion Round-Trip')
        print('─' * 68)
        gps_poly  = local_to_gps(polygon_coords, *gps_origin)
        roundtrip = gps_to_local(gps_poly, *gps_origin)
        max_err   = max(
            math.hypot(rx - lx, ry - ly)
            for (lx, ly), (rx, ry) in zip(polygon_coords, roundtrip)
        )
        check('GPS ↔ ENU round-trip error < 0.01 m',
              'pass' if max_err < 0.01 else 'fail',
              f'max_err = {max_err:.5f} m')
        print()

    # ----------------------------------------------------------------
    # Section 6: Visual output
    # ----------------------------------------------------------------
    print('─' * 68)
    print('  [6] Generating visual comparison...')
    print('─' * 68)
    _plot_comparison(polygon_coords, strategies, planner)
    print()

    print('=' * 68)
    print('  Done. Check coverage_efficiency.png for the visual output.')
    print('  If all checks above are PASS/WARN, you are ready for')
    print('  Gazebo SITL integration. Do NOT integrate detection until')
    print('  coverage is confirmed correct here first.')
    print('=' * 68)
    print()


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def _plot_comparison(polygon_coords, strategies, planner):
    fig, axes = plt.subplots(1, 3, figsize=(21, 7))
    poly = Polygon(polygon_coords)
    px, py = poly.exterior.xy

    strategy_list = [
        ('Lawnmower',  '#E24B4A'),
        ('Non-convex', '#1D9E75'),
        ('Spiral',     '#7F77DD'),
    ]

    for ax, (name, colour) in zip(axes, strategy_list):
        s = strategies[name]
        wps = s['wps']
        fp  = s['fp']

        # Footprint overlay (sampled along full path — not just endpoints)
        fp_patches = [MplPolygon(corners, closed=True) for corners in fp]
        ax.add_collection(PatchCollection(
            fp_patches, alpha=0.07, facecolor='steelblue', edgecolor='none'
        ))

        # Coverage gap highlighting (red dots where polygon is not covered)
        fp_polys = [Polygon(c) for c in fp]
        fp_union = unary_union(fp_polys)
        minx, miny, maxx, maxy = poly.bounds
        gap_xs, gap_ys = [], []
        for x in np.arange(minx, maxx + 2, 2.0):
            for y in np.arange(miny, maxy + 2, 2.0):
                pt = Point(x, y)
                if poly.contains(pt) and not fp_union.contains(pt):
                    gap_xs.append(x)
                    gap_ys.append(y)
        if gap_xs:
            ax.scatter(gap_xs, gap_ys, s=4, color='red', alpha=0.4,
                       zorder=3, label='Coverage gap')

        # Search area
        ax.fill(px, py, alpha=0.08, color='steelblue')
        ax.plot(px, py, 'b-', linewidth=1.5)

        # Out-of-bounds waypoints
        if s['outside']:
            ox = [w[0] for w in s['outside']]
            oy = [w[1] for w in s['outside']]
            ax.scatter(ox, oy, s=60, color='orange', zorder=7,
                       marker='X', label='Out of bounds!')

        # Path
        xs = [w[0] for w in wps]
        ys = [w[1] for w in wps]
        ax.plot(xs, ys, '-', color=colour, linewidth=1.4, label='Path')

        # Start / end
        ax.plot(xs[0],  ys[0],  'go', markersize=9, label='Start', zorder=5)
        ax.plot(xs[-1], ys[-1], 'rs', markersize=9, label='End',   zorder=5)

        # Waypoint numbers
        for i, (x, y) in enumerate(wps):
            ax.annotate(
                str(i + 1), (x, y),
                fontsize=5.5, ha='center', color='#333', zorder=6,
                path_effects=[pe.withStroke(linewidth=1.5, foreground='white')]
            )

        cov_flag = '✓' if s['cov'] >= 95.0 else '✗'
        fly_s = s['fly_s']
        t_str = f'{fly_s:.0f}s' if fly_s < 60 else f'{fly_s/60:.1f}min'
        ax.set_title(
            f'{name}\n'
            f'{len(wps)} WPs | {s["dist_m"]:.0f}m | '
            f'{s["cov"]:.1f}% {cov_flag} | ~{t_str}',
            fontsize=9.5
        )
        ax.set_xlabel('X (metres)')
        ax.set_ylabel('Y (metres)')
        ax.set_aspect('equal')

        handles, labels = ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        by_label['Detection footprint'] = mpatches.Patch(
            facecolor='steelblue', alpha=0.25, label='Detection footprint'
        )
        ax.legend(by_label.values(), by_label.keys(), fontsize=7)
        ax.grid(True, alpha=0.2)

    plt.suptitle(
        f'Coverage Efficiency  |  alt={ALTITUDE_M}m  '
        f'strip={CoveragePlanner(ALTITUDE_M,HFOV_DEG,VFOV_DEG,OVERLAP).strip_width:.1f}m  '
        f'overlap={int(OVERLAP*100)}%  speed={DRONE_SPEED_MS}m/s',
        fontsize=12, y=1.02
    )
    plt.tight_layout()
    plt.savefig('coverage_efficiency.png', dpi=150, bbox_inches='tight')
    if '--save-only' not in sys.argv:
        plt.show()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description='Check coverage planning efficiency before ROS integration'
    )
    parser.add_argument('--yaml', metavar='FILE',
                        help='Load polygon from a YAML file '
                             '(same format as coverage_planner_node)')
    parser.add_argument('--gps', action='store_true',
                        help='Run with a demo GPS polygon '
                             '(IIT Indore area, converted to local ENU)')
    parser.add_argument('--save-only', action='store_true',
                        help='Save PNG without opening a window')
    args = parser.parse_args()

    if args.yaml:
        polygon, gps_origin, is_gps = load_yaml_polygon(args.yaml)
        title = f'Coverage Efficiency — {os.path.basename(args.yaml)}'
        run_report(polygon, gps_origin=gps_origin, title=title)

    elif args.gps:
        ORIGIN_LAT, ORIGIN_LON = 22.5195, 75.9278
        gps_polygon = [
            (22.5195, 75.9278),
            (22.5204, 75.9278),
            (22.5204, 75.9290),
            (22.5195, 75.9290),
        ]
        polygon = gps_to_local(gps_polygon, ORIGIN_LAT, ORIGIN_LON)
        run_report(polygon, gps_origin=(ORIGIN_LAT, ORIGIN_LON),
                   title='Coverage Efficiency — GPS Demo (IIT Indore)')

    else:
        run_report(DEMO_POLYGON, title='Coverage Efficiency — Demo L-shape')


if __name__ == '__main__':
    main()
