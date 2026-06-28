#!/usr/bin/env python3
"""
check_iiti_polygon.py — Pre-flight polygon verification (no ROS needed)
========================================================================
Run this before launching SITL to confirm:
  1. The YAML loads cleanly
  2. GPS → local conversion is sane
  3. Coverage planner produces ≥95% with your chosen strategy
  4. A matplotlib plot is saved so you can visually inspect the path

Usage:
    cd ~/sar_ws/src/coverage_planner
    python3 check_iiti_polygon.py
    python3 check_iiti_polygon.py --yaml config/iiti_campus.yaml
    python3 check_iiti_polygon.py --yaml config/iiti_sar_zone.yaml --strategy spiral
"""

import sys
import os
import math
import argparse
import yaml

# Allow running from the package root without installing
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'coverage_planner'))

try:
    from planner import (
        CoveragePlanner,
        gps_to_local,
        local_to_gps,
        compute_coverage_pct,
        _path_length,
    )
except ImportError:
    print('[ERROR] Cannot import planner.py — run from coverage_planner package root.')
    sys.exit(1)

try:
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    from matplotlib.patches import Polygon as MplPolygon
    from matplotlib.collections import PatchCollection
    HAS_MPL = True
except ImportError:
    HAS_MPL = False
    print('[WARN] matplotlib not found — skipping plot.')

# ── CLI args ──────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser()
parser.add_argument(
    '--yaml', default='config/iiti_sar_zone.yaml',
    help='Path to polygon YAML file',
)
parser.add_argument(
    '--strategy', default='nonconvex',
    choices=['lawnmower', 'nonconvex', 'spiral'],
    help='Coverage strategy',
)
parser.add_argument(
    '--altitude', type=float, default=30.0,
    help='Mission altitude in metres',
)
parser.add_argument(
    '--hfov', type=float, default=60.0,
    help='Camera HFOV in degrees',
)
parser.add_argument(
    '--vfov', type=float, default=60.0,
    help='Camera VFOV in degrees',
)
parser.add_argument(
    '--overlap', type=float, default=0.1,
    help='Strip overlap fraction',
)
parser.add_argument(
    '--save-only', action='store_true',
    help='Save plot without showing (for headless runs)',
)
args = parser.parse_args()

print('=' * 68)
print('  IIT Indore Coverage Polygon Checker')
print('=' * 68)

# ── Load YAML ─────────────────────────────────────────────────────────────
yaml_path = args.yaml
if not os.path.isfile(yaml_path):
    print(f'[ERROR] YAML not found: {yaml_path}')
    sys.exit(1)

with open(yaml_path) as f:
    data = yaml.safe_load(f)

print(f'\n  YAML       : {yaml_path}')

if 'gps_polygon' in data:
    origin_lat = data.get('origin_lat')
    origin_lon = data.get('origin_lon')
    if not origin_lat or not origin_lon:
        print('[ERROR] gps_polygon requires origin_lat and origin_lon in YAML.')
        sys.exit(1)
    gps_poly = [tuple(p) for p in data['gps_polygon']]
    polygon  = gps_to_local(gps_poly, origin_lat, origin_lon)
    print(f'  Type       : GPS polygon ({len(gps_poly)} vertices)')
    print(f'  Origin     : {origin_lat:.6f}°N, {origin_lon:.6f}°E')

    # Round-trip validation
    roundtrip = local_to_gps(polygon, origin_lat, origin_lon)
    max_err = max(
        math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
        for a, b in zip(polygon, gps_to_local(roundtrip, origin_lat, origin_lon))
    )
    status = '✓ PASS' if max_err < 0.05 else '✗ FAIL'
    print(f'  GPS round-trip error: {max_err:.4f} m  [{status}]')

elif 'polygon' in data:
    polygon = [tuple(p) for p in data['polygon']]
    print(f'  Type       : Local polygon ({len(polygon)} vertices)')
else:
    print('[ERROR] YAML has neither "polygon" nor "gps_polygon" key.')
    sys.exit(1)

# ── Polygon stats ─────────────────────────────────────────────────────────
from shapely.geometry import Polygon as ShapelyPoly
sp = ShapelyPoly(polygon)
print(f'\n  Bounding box: {sp.bounds[0]:.1f}, {sp.bounds[1]:.1f}  →  '
      f'{sp.bounds[2]:.1f}, {sp.bounds[3]:.1f}  (x_min,y_min → x_max,y_max)')
print(f'  Width × Height: {sp.bounds[2]-sp.bounds[0]:.1f} m × '
      f'{sp.bounds[3]-sp.bounds[1]:.1f} m')
print(f'  Area          : {sp.area:.0f} m²  ({sp.area/10000:.2f} ha)')

# ── Run planner ───────────────────────────────────────────────────────────
planner = CoveragePlanner(
    altitude=args.altitude,
    hfov_deg=args.hfov,
    vfov_deg=args.vfov,
    overlap=args.overlap,
)

print(f'\n  Strategy   : {args.strategy}')
print(f'  Altitude   : {args.altitude} m')
print(f'  Strip width: {planner.strip_width:.2f} m')
print(f'  Safe width : {planner.safe_width:.2f} m')

print('\n  Planning…', end=' ', flush=True)
import time
t0 = time.time()
if args.strategy == 'nonconvex':
    wps = planner.generate_path_nonconvex(polygon)
elif args.strategy == 'spiral':
    wps = planner.generate_spiral_path(polygon)
else:
    wps = planner.generate_path(polygon)
t_plan = time.time() - t0
print(f'done in {t_plan*1000:.1f} ms')

footprints = planner.get_detection_footprints(wps)

print('  Computing coverage (grid sampling)…', end=' ', flush=True)
t0 = time.time()
pct = compute_coverage_pct(footprints, polygon, grid_step=2.0)
t_cov = time.time() - t0
print(f'done in {t_cov*1000:.1f} ms')

path_len = _path_length(wps)
n_strips = sum(
    1 for i in range(1, len(wps))
    if abs(wps[i][1] - wps[i-1][1]) > planner.safe_width * 0.5
) + 1

print(f'\n{"─"*50}')
print(f'  Waypoints        : {len(wps)}')
print(f'  Path length      : {path_len:.1f} m  ({path_len/1000:.3f} km)')
print(f'  Est. flight time : {path_len/5/60:.1f} min  (at 5 m/s)')
print(f'  Coverage         : {pct:.1f}%')
print(f'{"─"*50}')

# ── Pass/fail assessment ──────────────────────────────────────────────────
passed = True
checks = [
    ('Coverage ≥ 95%  (PS mid-eval target)',  pct >= 95.0,  f'{pct:.1f}%'),
    ('Coverage ≥ 99%  (PS end-eval target)',  pct >= 99.0,  f'{pct:.1f}%'),
    ('Planning time < 5 s',                   t_plan < 5.0, f'{t_plan*1000:.1f} ms'),
    ('At least 4 waypoints',                  len(wps) >= 4, str(len(wps))),
    ('Path length > 0',                       path_len > 0,  f'{path_len:.1f} m'),
]
print()
for desc, ok, val in checks:
    sym = '✓ PASS' if ok else '✗ FAIL'
    print(f'  {sym}  {desc}  [{val}]')
    if not ok:
        passed = False

print()
if passed:
    print('  ✓ All checks passed — safe to launch SITL.')
else:
    print('  ✗ Some checks failed — review before SITL.')

# ── Human detection ground truth ──────────────────────────────────────────
# Local positions of the 5 humans placed in iiti_sar.world
human_positions = [
    (80,  60,  'H1 (academic block)'),
    (180, 120, 'H2 (open ground)'),
    (280, 200, 'H3 (NE area)'),
    (120, 280, 'H4 (NW area)'),
    (220, 50,  'H5 (SE corner)'),
]
print('\n  Human ground-truth positions (local ENU from YAML origin):')
for hx, hy, label in human_positions:
    inside = sp.contains(ShapelyPoly([(hx-0.5,hy-0.5),(hx+0.5,hy-0.5),
                                      (hx+0.5,hy+0.5),(hx-0.5,hy+0.5)]))
    sym = '✓' if inside else '✗ OOB'
    print(f'    {sym}  {label:30s}  ({hx:6.1f}, {hy:6.1f}) m')

# ── Plot ──────────────────────────────────────────────────────────────────
if HAS_MPL:
    fig, ax = plt.subplots(figsize=(12, 9))
    px, py = sp.exterior.xy

    # Footprints
    fp_patches = [MplPolygon(corners, closed=True) for corners in footprints]
    ax.add_collection(PatchCollection(
        fp_patches, alpha=0.07, facecolor='steelblue', edgecolor='none'
    ))

    # Polygon
    ax.fill(px, py, alpha=0.12, color='steelblue')
    ax.plot(px, py, 'b-', linewidth=2, label='Search area')

    # Path
    xs = [w[0] for w in wps]
    ys = [w[1] for w in wps]
    ax.plot(xs, ys, '-', color='#E24B4A', linewidth=1.5, label='Coverage path')
    ax.plot(xs[0],  ys[0],  'go', markersize=10, label='Start', zorder=6)
    ax.plot(xs[-1], ys[-1], 'rs', markersize=10, label='End',   zorder=6)

    # Humans
    for hx, hy, label in human_positions:
        ax.plot(hx, hy, 'k^', markersize=12, zorder=7)
        ax.annotate(label.split('(')[0].strip(), (hx, hy),
                    textcoords='offset points', xytext=(6, 6), fontsize=8)

    # Strip guide lines
    sw = planner.safe_width
    for y in range(int(sp.bounds[1]), int(sp.bounds[3]) + int(sw), int(sw)):
        ax.axhline(y=y, color='lightgray', linewidth=0.4, linestyle='--')

    ax.set_title(
        f'IIT Indore SAR Zone — {args.strategy.capitalize()} | '
        f'{len(wps)} WPs | {path_len:.0f} m | {pct:.1f}% covered\n'
        f'Altitude={args.altitude} m | strip={planner.strip_width:.1f} m | '
        f'overlap={int(args.overlap*100)}%',
        fontsize=11,
    )
    ax.set_xlabel('X (metres, East from origin)')
    ax.set_ylabel('Y (metres, North from origin)')
    ax.set_aspect('equal')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    fp_patch = mpatches.Patch(facecolor='steelblue', alpha=0.3,
                               label='Detection footprint')
    handles, labels = ax.get_legend_handles_labels()
    ax.legend(handles + [fp_patch], labels + ['Detection footprint'], fontsize=9)

    out = 'iiti_coverage_check.png'
    plt.savefig(out, dpi=150, bbox_inches='tight')
    print(f'\n  Saved plot → {out}')
    if not args.save_only:
        plt.show()
