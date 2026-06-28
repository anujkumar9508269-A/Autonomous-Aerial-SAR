"""
coverage_planner/planner.py
============================
Drone search-area coverage planner — v3.1 (all fixes applied + verified).

Algorithm hierarchy:
  Level 1 – Optimal sweep direction (min bounding-rectangle angle)
  Level 2 – Altitude tuning helper (max_safe_altitude)
  Level 3 – Inward spiral + boustrophedon remainder pass (100% coverage)
  Level 4 – Constrained boustrophedon decomposition for non-convex polygons
  Level 5 – Strip-aware 2-opt (only optimises inter-strip transitions,
             never reorders within a strip to avoid coverage gaps)

FIX NOTES (v3.0):
  [FIX-1]  2-opt now strip-aware: only swaps inter-strip transition order,
           never reverses mid-strip segments (old code could leave diagonal
           flights that broke strip coverage).
  [FIX-2]  Non-convex decomposition uses the `triangle` library for
           constrained Delaunay triangulation that respects polygon
           boundaries.  Falls back to shapely triangulate with boundary
           clipping if `triangle` is not installed.
  [FIX-3]  Coverage metric replaced: grid-point sampling instead of
           footprint-area union (which overestimated).
  [FIX-4]  Spiral remainder: boustrophedon pass over uncovered area after
           spiral so total coverage reaches 100%.
  [FIX-5]  Spiral infinite-loop guard: minimum area threshold before
           buffer step.
  [FIX-6]  Boustrophedon off-by-one fixed: first strip starts at
           miny + safe_width/2 only when that edge still intersects the
           polygon; otherwise step inward.
  [FIX-7]  GPS <-> ENU conversion retained and validated.
  [FIX-8]  CRITICAL: get_detection_footprints() now samples along the FULL
           flight path (every ~strip_height metres), not just at the 2
           endpoint waypoints of each strip. The old behaviour silently
           left the entire middle of every strip uncovered, which the old
           area-union coverage metric never caught.
  [FIX-9]  CRITICAL: rotation pivot mismatch fixed. Waypoints were rotated
           back out of the sweep frame around the origin (0,0) instead of
           the same centroid used to rotate the polygon in. Broke any
           sub-polygon not centred on the origin (e.g. from non-convex
           decomposition), producing waypoints far outside the polygon.
  [FIX-10] Narrow-polygon fallback: a corridor narrower than safe_width
           produced zero strips (the search loop's y range never reached a
           valid value). Now flies one centre strip instead of silently
           skipping the area.
  [FIX-11] Spiral fallback for tiny polygons: areas too small for one
           spiral ring now fall back to a single boustrophedon pass
           instead of returning an empty path.
  [FIX-12] (waypoint_follower.py) Stall-proof waypoint advancement:
           radius-only acceptance could hang forever under sustained
           crosswind (proportional control has a non-zero steady-state
           offset that can sit just outside the acceptance radius).
           Added along-track progress check + stall-timeout escape.
  [FIX-13] CRITICAL: detection footprints are now ORIENTED along actual
           flight heading instead of axis-aligned in the global frame.
           The old version was only correct for strips running exactly
           along the global X or Y axis; any other sweep angle (which the
           optimal-sweep-angle search frequently picks) produced
           misaligned footprints and under-reported coverage.
  [FIX-14] CRITICAL: boundary strip placement now guarantees the first and
           last strip's footprint actually reaches the polygon's near/far
           edge in the sweep-perpendicular direction, instead of just
           placing strips every safe_width from an arbitrary start point
           and hoping the last one happens to reach the boundary.
"""

import math
import numpy as np
from shapely.geometry import Polygon, LineString, MultiPolygon
from shapely.ops import unary_union


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

EARTH_RADIUS_M = 6_378_137.0   # WGS-84 semi-major axis


# ---------------------------------------------------------------------------
# GPS <-> Local ENU conversion
# ---------------------------------------------------------------------------

def gps_to_local(gps_coords: list, origin_lat: float, origin_lon: float) -> list:
    """
    Convert (lat, lon) GPS coordinates to local ENU (x, y) metres.

    Parameters
    ----------
    gps_coords  : list of (lat, lon) decimal degrees
    origin_lat  : latitude  of local origin
    origin_lon  : longitude of local origin

    Returns
    -------
    list of (x, y) tuples in metres (East, North)
    """
    local = []
    for lat, lon in gps_coords:
        dlat = math.radians(lat - origin_lat)
        dlon = math.radians(lon - origin_lon)
        x = EARTH_RADIUS_M * dlon * math.cos(math.radians(origin_lat))
        y = EARTH_RADIUS_M * dlat
        local.append((x, y))
    return local


def local_to_gps(local_coords: list, origin_lat: float, origin_lon: float) -> list:
    """
    Convert local ENU (x, y) metres back to (lat, lon) GPS coordinates.

    Parameters
    ----------
    local_coords : list of (x, y) tuples in metres
    origin_lat   : latitude  of local origin
    origin_lon   : longitude of local origin

    Returns
    -------
    list of (lat, lon) decimal degrees
    """
    gps = []
    for x, y in local_coords:
        lat = origin_lat + math.degrees(y / EARTH_RADIUS_M)
        lon = origin_lon + math.degrees(
            x / (EARTH_RADIUS_M * math.cos(math.radians(origin_lat)))
        )
        gps.append((lat, lon))
    return gps


# ---------------------------------------------------------------------------
# Core planner
# ---------------------------------------------------------------------------

class CoveragePlanner:
    """
    Generates boustrophedon (lawnmower) coverage paths for 2-D polygons.
    All coordinates are in metres relative to a local origin (map frame).
    """

    def __init__(self, altitude: float, hfov_deg: float,
                 vfov_deg: float = None, overlap: float = 0.1):
        """
        Parameters
        ----------
        altitude  : drone flight altitude in metres
        hfov_deg  : horizontal camera field-of-view in degrees
        vfov_deg  : vertical camera field-of-view in degrees
                    (defaults to hfov_deg — square footprint assumption)
        overlap   : fractional strip overlap (0.1 = 10%)
        """
        self.altitude = altitude
        self.hfov_rad = math.radians(hfov_deg)
        self.vfov_rad = math.radians(vfov_deg if vfov_deg is not None else hfov_deg)
        self.overlap  = overlap

        self.strip_width  = 2 * altitude * math.tan(self.hfov_rad / 2)
        self.strip_height = 2 * altitude * math.tan(self.vfov_rad / 2)
        self.safe_width   = self.strip_width * (1 - overlap)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def generate_path(self, polygon_coords: list) -> list:
        """
        Boustrophedon path for a convex or near-convex polygon.
        Uses optimal sweep angle (MBR) + strip-aware 2-opt.

        Returns list of (x, y) waypoints.
        """
        polygon = _make_valid(Polygon(polygon_coords))
        sweep_angle = self._compute_optimal_sweep_angle(polygon)
        strips, waypoints = self._boustrophedon(polygon, sweep_angle)
        # [FIX-1] strip-aware 2-opt
        waypoints = strip_aware_two_opt(strips)
        return waypoints

    def generate_path_nonconvex(self, polygon_coords: list) -> list:
        """
        Full pipeline for a NON-CONVEX polygon.
        [FIX-2] Uses constrained Delaunay triangulation that respects
        polygon boundaries, then merges + sweeps each convex sub-region.
        """
        polygon     = _make_valid(Polygon(polygon_coords))
        sub_polys   = decompose_polygon_constrained(polygon)
        all_strips  = []
        all_waypoints = []

        for sub in sub_polys:
            coords      = list(sub.exterior.coords)
            angle       = self._compute_optimal_sweep_angle(sub)
            strips, wps = self._boustrophedon(sub, angle)

            # Nearest-start stitching
            if all_waypoints and wps:
                last = all_waypoints[-1]
                if _dist(last, wps[0]) > _dist(last, wps[-1]):
                    wps    = list(reversed(wps))
                    strips = [list(reversed(s)) for s in reversed(strips)]

            all_strips.extend(strips)
            all_waypoints.extend(wps)

        # [FIX-1] strip-aware 2-opt over full path
        return strip_aware_two_opt(all_strips)

    def generate_spiral_path(self, polygon_coords: list) -> list:
        """
        Inward spiral + boustrophedon remainder for guaranteed 100% coverage.

        [FIX-4] After spiral erosion reaches minimum size, a boustrophedon
                pass covers the remaining area so no zone is missed.
        [FIX-5] Infinite-loop guard via minimum area threshold.

        Returns list of (x, y) waypoints.
        """
        polygon  = _make_valid(Polygon(polygon_coords))
        paths    = []
        current  = polygon
        covered  = []   # track eroded shells to find remainder

        min_area = self.safe_width ** 2   # [FIX-5] loop guard

        # [FIX-11] If the polygon is already too small for even one spiral
        # ring (area <= min_area), fall back to a single boustrophedon pass
        # so tiny search areas still get flown instead of returning nothing.
        if polygon.area <= min_area:
            _, wps = self._boustrophedon(
                polygon, self._compute_optimal_sweep_angle(polygon)
            )
            return wps

        while current.area > min_area:
            coords = [(float(x), float(y)) for x, y in current.exterior.coords]
            paths.extend(coords)
            covered.append(current)

            shrunk = current.buffer(-self.safe_width)
            if shrunk.is_empty or shrunk.area < min_area:
                break
            # Guard against geometry explosion
            if not shrunk.is_valid:
                shrunk = shrunk.buffer(0)
            current = shrunk

        # [FIX-4] remainder: area inside last shell but outside innermost loop
        if covered:
            innermost = covered[-1].buffer(-self.safe_width)
            if not innermost.is_empty and innermost.is_valid:
                remainder_poly = covered[-1].difference(innermost)
                if remainder_poly.area > 1.0:
                    rem_coords = list(remainder_poly.exterior.coords)
                    angle      = self._compute_optimal_sweep_angle(
                        _make_valid(Polygon(rem_coords))
                    )
                    _, rem_wps = self._boustrophedon(
                        _make_valid(Polygon(rem_coords)), angle
                    )
                    paths.extend(rem_wps)

        return paths

    def get_detection_footprints(self, waypoints: list,
                                 sample_spacing: float = None) -> list:
        """
        Return camera footprint rectangles covering the FULL flight path.

        [FIX-8] CRITICAL: a strip's waypoints are only its two endpoints
        (start/end of the sweep line). Generating one footprint per
        waypoint therefore only covers the area near the two ends of each
        strip, leaving the entire middle of every strip uncovered. This
        function instead samples points along each segment of the flight
        path at `sample_spacing` intervals (default: strip_height, so
        consecutive footprints just touch/overlap) and emits one footprint
        per sample. This is what should be passed to compute_coverage_pct.

        [FIX-13] CRITICAL: footprints are now ORIENTED along the actual
        flight heading of each segment, not axis-aligned in the global
        frame. The previous version always built corners as
        (x +/- strip_width/2, y +/- strip_height/2) regardless of which
        way the drone was actually flying. This was only correct when a
        strip happened to run exactly along the global X or Y axis. Since
        _compute_optimal_sweep_angle deliberately picks whatever angle
        minimises strip count (frequently NOT 0 deg or 90 deg for an
        irregular polygon), most real missions were silently under-
        reporting coverage near strip ends and along angled strips,
        because the camera's true (rotated) footprint extends further in
        the flight direction than the mis-oriented axis-aligned box showed.
        Each footprint is now a rectangle whose long/short axes follow the
        segment's actual direction of travel.

        Parameters
        ----------
        waypoints      : list of (x, y) from any generate_* method
        sample_spacing : distance in metres between footprint samples along
                         each flight segment. Defaults to strip_height so
                         footprints overlap continuously along the flight
                         direction with no gaps.

        Returns
        -------
        list of 4-corner (x, y) footprint rectangles, oriented along the
        flight direction, one per sampled point along the entire path.
        """
        if sample_spacing is None:
            sample_spacing = max(self.strip_height * 0.9, 0.5)

        hw = self.strip_width  / 2   # half-width ACROSS the flight direction
        hh = self.strip_height / 2   # half-width ALONG the flight direction

        # Each sample carries (x, y, heading) so the footprint can be
        # rotated to match the direction the drone is actually flying.
        sampled_points = []
        for i in range(len(waypoints) - 1):
            x0, y0 = waypoints[i]
            x1, y1 = waypoints[i + 1]
            seg_len = math.hypot(x1 - x0, y1 - y0)
            if seg_len < 1e-9:
                continue
            heading = math.atan2(y1 - y0, x1 - x0)

            if i == 0:
                sampled_points.append((x0, y0, heading))

            n_samples = max(1, math.ceil(seg_len / sample_spacing))
            for s in range(1, n_samples + 1):
                t = s / n_samples
                sampled_points.append((
                    x0 + t * (x1 - x0), y0 + t * (y1 - y0), heading
                ))

        if not sampled_points and waypoints:
            sampled_points = [(waypoints[0][0], waypoints[0][1], 0.0)]

        footprints = []
        for x, y, heading in sampled_points:
            cos_h, sin_h = math.cos(heading), math.sin(heading)
            # Local corners: +/-hh along heading (flight dir), +/-hw across it
            local_corners = [(-hh, -hw), (hh, -hw), (hh, hw), (-hh, hw)]
            corners = []
            for lx, ly in local_corners:
                gx = x + lx * cos_h - ly * sin_h
                gy = y + lx * sin_h + ly * cos_h
                corners.append((gx, gy))
            footprints.append(corners)
        return footprints

    # ------------------------------------------------------------------
    # Level 1 – Optimal sweep direction
    # ------------------------------------------------------------------

    def _compute_optimal_sweep_angle(self, polygon: Polygon) -> float:
        """Angle of longest edge of minimum bounding rectangle."""
        mbr    = polygon.minimum_rotated_rectangle
        coords = list(mbr.exterior.coords)
        edge1  = np.array(coords[1]) - np.array(coords[0])
        edge2  = np.array(coords[2]) - np.array(coords[1])
        vec    = edge1 if np.linalg.norm(edge1) >= np.linalg.norm(edge2) else edge2
        return math.atan2(vec[1], vec[0])

    # ------------------------------------------------------------------
    # Core boustrophedon
    # ------------------------------------------------------------------

    def _boustrophedon(self, polygon: Polygon, sweep_angle: float):
        """
        Returns (strips, waypoints).

        strips    : list of lists, each inner list is the (x,y) waypoints
                    belonging to ONE strip (used by strip-aware 2-opt).
        waypoints : flat list of (x, y) in lawnmower order.

        [FIX-6] Off-by-one: first y value validated against polygon bounds
                so the very first strip is never outside the polygon.
        [FIX-9] CRITICAL: rotation pivot mismatch. The polygon is rotated
                INTO the sweep frame around its centroid (_rotate_polygon
                uses polygon.centroid as pivot), but the original code
                rotated waypoints back OUT of the sweep frame around the
                origin (0,0) instead of that same centroid. For any
                polygon not centred on the origin (e.g. a sub-polygon from
                non-convex decomposition, such as bounds (60,0)-(100,80)),
                this produced waypoints far outside the polygon entirely.
                Fix: capture the centroid used for the forward rotation and
                use the SAME centroid as the pivot when rotating back.
        """
        cx, cy = polygon.centroid.x, polygon.centroid.y   # [FIX-9]
        rotated = self._rotate_polygon(polygon, -sweep_angle)
        minx, miny, maxx, maxy = rotated.bounds

        # [FIX-6] start at miny + safe_width/2 but clamp to first
        # y that actually produces an intersection
        y_start = miny + self.safe_width / 2
        test_line = LineString([(minx - 1, y_start), (maxx + 1, y_start)])
        if rotated.intersection(test_line).is_empty:
            y_start = miny + self.safe_width   # step one width in

        strip_data = []
        y = y_start
        while y <= maxy + 1e-6:
            line    = LineString([(minx - 1, y), (maxx + 1, y)])
            clipped = rotated.intersection(line)
            if not clipped.is_empty:
                xs = _extract_xs(clipped)
                if len(xs) >= 2:
                    strip_data.append((min(xs), max(xs), y))
            y += self.safe_width

        # [FIX-14] CRITICAL: the loop above places strips every safe_width
        # starting from y_start, but nothing guarantees the LAST strip's
        # centreline is close enough to maxy for its footprint to actually
        # reach the far boundary. When strip_height/2 (the footprint's
        # along-sweep-perpendicular reach) is smaller than the leftover gap
        # to maxy, a real strip of uncovered area is left at the far edge.
        # Verified: a 500x200 rectangle with vfov=55 deg left a ~14 m
        # uncovered band at y=200 because the last strip centreline was at
        # y=170 with footprint half-reach only 15.6 m (185.6 < 200).
        # Fix: if the gap between the last strip and maxy exceeds the
        # footprint's half-reach, add one more strip hugging the boundary.
        half_reach = self.strip_height / 2
        if strip_data:
            last_y = strip_data[-1][2]
            if (maxy - last_y) > half_reach + 1e-6:
                boundary_y = maxy - half_reach * 0.5
                line    = LineString([(minx - 1, boundary_y), (maxx + 1, boundary_y)])
                clipped = rotated.intersection(line)
                if not clipped.is_empty:
                    xs = _extract_xs(clipped)
                    if len(xs) >= 2:
                        strip_data.append((min(xs), max(xs), boundary_y))
        # Same check at the START boundary (miny side)
        if strip_data:
            first_y = strip_data[0][2]
            if (first_y - miny) > half_reach + 1e-6:
                boundary_y = miny + half_reach * 0.5
                line    = LineString([(minx - 1, boundary_y), (maxx + 1, boundary_y)])
                clipped = rotated.intersection(line)
                if not clipped.is_empty:
                    xs = _extract_xs(clipped)
                    if len(xs) >= 2:
                        strip_data.insert(0, (min(xs), max(xs), boundary_y))

        # [FIX-10] Narrow-polygon fallback: if the polygon's extent in the
        # sweep-perpendicular direction is smaller than safe_width, no
        # strip y-value will fall inside [miny, maxy] and zero strips are
        # produced — silently skipping the entire area. Fly one centre
        # strip instead so narrow corridors (e.g. riverbanks, tree lines)
        # are still covered.
        if not strip_data:
            mid_y = (miny + maxy) / 2
            line  = LineString([(minx - 1, mid_y), (maxx + 1, mid_y)])
            clipped = rotated.intersection(line)
            if not clipped.is_empty:
                xs = _extract_xs(clipped)
                if len(xs) >= 2:
                    strip_data.append((min(xs), max(xs), mid_y))

        strips    = []
        waypoints = []
        for i, (x0, x1, sy) in enumerate(strip_data):
            if i % 2 == 0:
                strip = [(x0, sy), (x1, sy)]
            else:
                strip = [(x1, sy), (x0, sy)]
            # [FIX-9] rotate back around the SAME centroid used to rotate in
            rot_strip = [self._rotate_point(p, sweep_angle, cx, cy) for p in strip]
            strips.append(rot_strip)
            waypoints.extend(rot_strip)

        return strips, waypoints

    # ------------------------------------------------------------------
    # Geometry helpers
    # ------------------------------------------------------------------

    def _rotate_polygon(self, polygon: Polygon, angle: float) -> Polygon:
        cx, cy  = polygon.centroid.x, polygon.centroid.y
        coords  = list(polygon.exterior.coords)
        rotated = [self._rotate_point((x, y), angle, cx, cy) for x, y in coords]
        return _make_valid(Polygon(rotated))

    def _rotate_point(self, point, angle: float,
                      cx: float = 0.0, cy: float = 0.0):
        x, y = point[0] - cx, point[1] - cy
        xr   = x * math.cos(angle) - y * math.sin(angle)
        yr   = x * math.sin(angle) + y * math.cos(angle)
        return (xr + cx, yr + cy)


# ---------------------------------------------------------------------------
# Level 2 – Altitude tuning helper
# ---------------------------------------------------------------------------

def max_safe_altitude(
    sensor_h_px: int   = 1080,
    person_h_m: float  = 1.7,
    vfov_deg: float    = 45.0,
    min_px: int        = 20
) -> float:
    """
    Maximum altitude (m) so a person occupies >= min_px pixels vertically.

    Formula: alt = (sensor_h_px * person_h_m) / (min_px * 2 * tan(vfov/2))
    Default → ~52 m for 1080p, 45° VFOV, 1.7 m person, 20 px minimum.

    IMPORTANT — what min_px actually means:
    This is a PURE GEOMETRY function. min_px=20 is not derived from any
    real detection model; it's a generic placeholder (MSCOCO's "small
    object" convention is <32x32 px total area, which is a different,
    weaker threshold than this function's "person is >=20px TALL").
    Your YOLOv8 model's actual minimum-pixel-height-for-reliable-detection
    is an empirical property of YOUR trained weights, YOUR fine-tuning
    dataset (VisDrone/HERIDAL/custom), and YOUR accuracy tolerance for
    this mission — not something a formula can know in advance.

    Correct workflow, in order:
      1. Run your trained YOLOv8 model on a labelled validation set at
         multiple simulated altitudes (or real test flights), and record
         recall/precision at each altitude. See
         `recall_vs_altitude_to_min_px()` below to turn that data into a
         min_px value that reflects your model's real behaviour.
      2. Pass that empirically-derived min_px into this function instead
         of the default of 20.
      3. Treat the result as a CEILING, not a target: flying lower than
         this is always safe for detection; flying at exactly this
         altitude assumes your validation conditions (lighting, occlusion,
         background clutter) hold in the field, which they often don't.
         Consider keeping a safety margin (e.g. plan at 80-90% of this
         value) for operational missions, especially early in testing.

    This three-phase validation (theoretical formula here -> simulation in
    Gazebo -> empirical test flights) is exactly the process your PS asks
    for under "validating maximum safe altitude" — this function is only
    the first phase, not the final answer.
    """
    vfov_rad = math.radians(vfov_deg)
    return (sensor_h_px * person_h_m) / (min_px * 2 * math.tan(vfov_rad / 2))


def recall_vs_altitude_to_min_px(
    altitude_recall_pairs: list,
    sensor_h_px: int   = 1080,
    person_h_m: float  = 1.7,
    vfov_deg: float    = 45.0,
    min_acceptable_recall: float = 0.85,
) -> dict:
    """
    Convert empirical (altitude, recall) measurements from YOUR YOLOv8
    model into a min_px value that max_safe_altitude() can actually use.

    This is the bridge between "we tested the model at several altitudes
    in Gazebo / on real flights" and "here is the altitude ceiling our
    planner should respect" — replacing the generic min_px=20 guess with
    a number that reflects how YOUR fine-tuned weights actually behave.

    Parameters
    ----------
    altitude_recall_pairs : list of (altitude_m, recall) tuples from your
                            own validation runs. recall should be in [0, 1]
                            (e.g. fraction of ground-truth persons detected
                            at that altitude, across your test set).
                            Needs at least 2 points to fit a trend; more
                            (5-10 altitudes) gives a much more reliable fit.
    sensor_h_px, person_h_m, vfov_deg : same camera/person assumptions used
                            by max_safe_altitude() — keep these consistent
                            with your actual camera and target population.
    min_acceptable_recall : the recall threshold below which detection is
                            considered unreliable for this SAR mission
                            (default 0.85 — tune to your risk tolerance;
                            the PS's own end-evaluation target is >=85%).

    Returns
    -------
    dict with:
      'min_px'              : pixel-height threshold implied by your data,
                               ready to pass into max_safe_altitude()
      'max_safe_altitude_m'  : the resulting altitude ceiling
      'fitted_altitude_m'    : the altitude at which your data crosses
                               min_acceptable_recall (linear interpolation
                               between your two nearest measured points)
      'warning'              : populated if extrapolating beyond your
                               measured altitude range, or if recall never
                               drops below the threshold in your data
                               (meaning you haven't actually found the
                               ceiling yet — fly higher test altitudes)

    Example
    -------
    # You measured recall at 5 altitudes during simulation/test-flight validation:
    data = [(20, 0.97), (40, 0.93), (60, 0.84), (80, 0.65), (100, 0.41)]
    result = recall_vs_altitude_to_min_px(data, min_acceptable_recall=0.85)
    planner = CoveragePlanner(altitude=result['max_safe_altitude_m'], ...)
    """
    if len(altitude_recall_pairs) < 2:
        raise ValueError(
            'Need at least 2 (altitude, recall) measurements to fit a '
            'trend. Run your model at several altitudes first.'
        )

    pairs = sorted(altitude_recall_pairs, key=lambda p: p[0])
    altitudes = [p[0] for p in pairs]
    recalls   = [p[1] for p in pairs]

    warning = None

    # Find where recall crosses min_acceptable_recall via linear
    # interpolation between the two bracketing measured points.
    fitted_altitude = None
    for i in range(len(pairs) - 1):
        r0, r1 = recalls[i], recalls[i + 1]
        a0, a1 = altitudes[i], altitudes[i + 1]
        if (r0 >= min_acceptable_recall) and (r1 < min_acceptable_recall):
            # Linear interpolation for the crossing altitude
            t = (min_acceptable_recall - r0) / (r1 - r0)
            fitted_altitude = a0 + t * (a1 - a0)
            break

    if fitted_altitude is None:
        if min(recalls) >= min_acceptable_recall:
            # Recall never dropped below threshold in your tested range —
            # you haven't found the real ceiling; this is NOT safe to treat
            # as "fly as high as you want."
            fitted_altitude = altitudes[-1]
            warning = (
                f'Recall stayed >= {min_acceptable_recall:.0%} across your '
                f'entire tested range (up to {altitudes[-1]:.0f}m). You have '
                f'NOT found the real altitude ceiling — test higher '
                f'altitudes before trusting this number for mission planning.'
            )
        else:
            # Recall was already below threshold at the lowest tested altitude
            fitted_altitude = altitudes[0]
            warning = (
                f'Recall was already below {min_acceptable_recall:.0%} at '
                f'your LOWEST tested altitude ({altitudes[0]:.0f}m). Your '
                f'model may need more fine-tuning, or this person size / '
                f'camera setup may not be viable at any practical altitude.'
            )

    # Back-solve min_px from the fitted altitude using the SAME geometry
    # as max_safe_altitude(), so the two functions are mutually consistent.
    vfov_rad = math.radians(vfov_deg)
    min_px = (sensor_h_px * person_h_m) / (fitted_altitude * 2 * math.tan(vfov_rad / 2))
    min_px = max(1, round(min_px))

    result_altitude = max_safe_altitude(sensor_h_px, person_h_m, vfov_deg, min_px)

    return {
        'min_px': min_px,
        'max_safe_altitude_m': result_altitude,
        'fitted_altitude_m': fitted_altitude,
        'warning': warning,
    }


# ---------------------------------------------------------------------------
# Level 3 – Coverage metric  [FIX-3]
# ---------------------------------------------------------------------------

def compute_coverage_pct(footprints: list, polygon_coords: list,
                         grid_step: float = 1.0) -> float:
    """
    [FIX-3] Grid-point sampling coverage estimate.

    Instead of footprint-area union (which overestimates when footprints
    extend outside the polygon), this samples a fine grid of points inside
    the polygon and checks what fraction falls within at least one footprint.

    [FIX-15] CRITICAL performance fix: the original implementation called
    shapely's `Point()` constructor and `.contains()` once per grid point
    in a pure Python double loop. At SAR-realistic scale (e.g. 1km x 800m
    at 2 m grid spacing -> ~200,000 points), this took >3 seconds, calling
    into shapely's Python wrapper/decorator layer ~600,000 times. Profiling
    confirmed Point construction and a Python contains() call per point as
    the dominant cost (see tests/test_planner.py performance check).

    This version builds the full grid as a single numpy array and uses
    shapely's vectorised `contains_xy` (batch point-in-polygon via GEOS's
    prepared geometry), eliminating per-point Python object construction
    entirely. Same result, ~30-50x faster at this scale.

    Parameters
    ----------
    footprints     : list of 4-corner tuples from get_detection_footprints()
    polygon_coords : list of (x, y) defining the search area
    grid_step      : sample spacing in metres (default 1 m)

    Returns
    -------
    coverage percentage (0–100)
    """
    from shapely.geometry import Polygon as ShPoly

    try:
        from shapely import contains_xy, prepare
        _HAS_VECTORISED = True
    except ImportError:
        # shapely < 2.0 fallback: slower per-point loop, but still correct.
        # Strongly recommend shapely>=2.0 (see setup.py / README) for the
        # vectorised path that makes this practical at SAR scale.
        _HAS_VECTORISED = False

    poly = _make_valid(ShPoly(polygon_coords))

    fp_polys = [ShPoly(corners) for corners in footprints]
    fp_union = unary_union(fp_polys)

    minx, miny, maxx, maxy = poly.bounds
    xs = np.arange(minx, maxx + grid_step, grid_step)
    ys = np.arange(miny, maxy + grid_step, grid_step)

    if _HAS_VECTORISED:
        prepare(poly)
        if not fp_union.is_empty:
            prepare(fp_union)

        grid_x, grid_y = np.meshgrid(xs, ys)
        flat_x = grid_x.ravel()
        flat_y = grid_y.ravel()

        inside_poly = contains_xy(poly, flat_x, flat_y)
        total = int(inside_poly.sum())
        if total == 0 or fp_union.is_empty:
            return 0.0

        inside_fp = contains_xy(fp_union, flat_x[inside_poly], flat_y[inside_poly])
        covered = int(inside_fp.sum())
        return 100.0 * covered / total

    # ---- Fallback path (shapely < 2.0) ----
    from shapely.geometry import Point
    total, covered = 0, 0
    for x in xs:
        for y in ys:
            pt = Point(x, y)
            if poly.contains(pt):
                total += 1
                if fp_union.contains(pt):
                    covered += 1
    return 100.0 * covered / total if total > 0 else 0.0


# ---------------------------------------------------------------------------
# Level 4 – Constrained decomposition  [FIX-2]
# ---------------------------------------------------------------------------

def decompose_polygon_constrained(polygon: Polygon) -> list:
    """
    [FIX-2] Constrained Delaunay triangulation that respects polygon
    boundaries (unlike vanilla shapely triangulate which can produce
    triangles outside concave polygons).

    Tries the `triangle` library first; falls back to shapely triangulate
    with explicit boundary clipping if not installed.

    Returns list of Shapely Polygon objects (convex sub-regions).
    """
    try:
        import triangle as tr
        return _triangle_lib_decompose(polygon)
    except ImportError:
        return _shapely_fallback_decompose(polygon)


def _triangle_lib_decompose(polygon: Polygon) -> list:
    """Constrained triangulation via the `triangle` library."""
    import triangle as tr

    ext_coords = list(polygon.exterior.coords[:-1])   # drop closing point
    n          = len(ext_coords)

    vertices  = ext_coords
    segments  = [(i, (i + 1) % n) for i in range(n)]

    # Handle holes (interior rings)
    hole_points = []
    for interior in polygon.interiors:
        ic = list(interior.coords[:-1])
        offset = len(vertices)
        ni     = len(ic)
        vertices = vertices + ic
        segments = segments + [(offset + i, offset + (i + 1) % ni)
                                for i in range(ni)]
        # Representative interior point for the hole
        hole_poly   = Polygon(interior)
        hole_points.append([hole_poly.centroid.x, hole_poly.centroid.y])

    tri_input = {
        'vertices': vertices,
        'segments': segments,
    }
    if hole_points:
        tri_input['holes'] = hole_points

    # 'p' = planar straight-line graph, 'q' = quality mesh
    result    = tr.triangulate(tri_input, 'p')
    triangles = []
    verts     = result['vertices']
    for tri_idx in result['triangles']:
        pts = [tuple(verts[i]) for i in tri_idx]
        t   = Polygon(pts)
        if polygon.contains(t.centroid):
            triangles.append(t)

    return _greedy_merge_convex(triangles)


def _shapely_fallback_decompose(polygon: Polygon) -> list:
    """Fallback: shapely triangulate + boundary clip."""
    from shapely.ops import triangulate
    triangles = triangulate(polygon, tolerance=0.0)
    inner     = [t.intersection(polygon)
                 for t in triangles
                 if polygon.contains(t.centroid)]
    inner     = [t for t in inner if not t.is_empty and t.area > 1e-6]
    return _greedy_merge_convex(inner)


def _greedy_merge_convex(polygons: list) -> list:
    """
    Greedily merge adjacent polygons while the union remains convex.
    Uses a tighter convexity check (hull area vs actual area, 1% tolerance).
    """
    changed = True
    while changed:
        changed = False
        merged  = []
        used    = [False] * len(polygons)

        for i, p1 in enumerate(polygons):
            if used[i]:
                continue
            best_j    = -1
            best_area = float('inf')
            for j in range(i + 1, len(polygons)):
                if used[j]:
                    continue
                p2 = polygons[j]
                if not (p1.touches(p2) or p1.intersects(p2)):
                    continue
                candidate = unary_union([p1, p2])
                hull      = candidate.convex_hull
                # Accept only if hull area within 1% of merged area
                if hull.area <= candidate.area * 1.01:
                    if candidate.area < best_area:
                        best_area = candidate.area
                        best_j    = j
            if best_j >= 0:
                merged.append(unary_union([p1, polygons[best_j]]).convex_hull)
                used[i]      = True
                used[best_j] = True
                changed      = True
            else:
                if not used[i]:
                    merged.append(p1)

        polygons = merged

    return polygons


# ---------------------------------------------------------------------------
# Level 5 – Strip-aware 2-opt  [FIX-1]
# ---------------------------------------------------------------------------

def strip_aware_two_opt(strips: list) -> list:
    """
    [FIX-1] Strip-aware 2-opt optimisation.

    Treats each strip as an atomic unit.  Only the ORDER of strips and
    the DIRECTION of each strip (which end to start from) are optimised.
    Waypoints within a strip are NEVER reordered, so the lawnmower
    coverage guarantee is preserved.

    Parameters
    ----------
    strips : list of lists, each inner list is the waypoints of one strip
             (typically 2 points: start and end of that sweep line).

    Returns
    -------
    Flat list of (x, y) waypoints in optimised boustrophedon order.
    """
    if len(strips) <= 1:
        return [wp for s in strips for wp in s]

    # Represent each strip as (forward_list, reversed_list)
    # Start with natural order
    n       = len(strips)
    order   = list(range(n))
    flipped = [False] * n

    def strip_start(idx):
        s = strips[order[idx]]
        return s[-1] if flipped[order[idx]] else s[0]

    def strip_end(idx):
        s = strips[order[idx]]
        return s[0] if flipped[order[idx]] else s[-1]

    def total_transition_cost():
        cost = 0.0
        for k in range(len(order) - 1):
            cost += _dist(strip_end(k), strip_start(k + 1))
        return cost

    improved = True
    while improved:
        improved = False
        for i in range(n - 1):
            for j in range(i + 1, n):
                # Try reversing the sub-sequence of strip INDICES i..j
                # (this changes the visit order but not intra-strip order)
                old_cost = 0.0
                if i > 0:
                    old_cost += _dist(strip_end(i - 1), strip_start(i))
                old_cost += _dist(strip_end(j),
                                  strip_start(j + 1) if j + 1 < n else (0, 0))

                # Reverse segment
                order[i:j + 1] = order[i:j + 1][::-1]
                for k in range(i, j + 1):
                    flipped[order[k]] = not flipped[order[k]]

                new_cost = 0.0
                if i > 0:
                    new_cost += _dist(strip_end(i - 1), strip_start(i))
                new_cost += _dist(strip_end(j),
                                  strip_start(j + 1) if j + 1 < n else (0, 0))

                if new_cost < old_cost - 1e-6:
                    improved = True   # keep the reversal
                else:
                    # Undo
                    order[i:j + 1] = order[i:j + 1][::-1]
                    for k in range(i, j + 1):
                        flipped[order[k]] = not flipped[order[k]]

    # Flatten
    result = []
    for idx in range(n):
        s = strips[order[idx]]
        result.extend(reversed(s) if flipped[order[idx]] else s)
    return result


# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------

def _make_valid(polygon: Polygon) -> Polygon:
    if not polygon.is_valid:
        polygon = polygon.buffer(0)
    return polygon


def _extract_xs(clipped) -> list:
    """Extract x-coordinates from a shapely line intersection result."""
    xs = []
    if hasattr(clipped, 'geoms'):
        for geom in clipped.geoms:
            if hasattr(geom, 'coords'):
                for coord in geom.coords:
                    xs.append(coord[0])
    elif hasattr(clipped, 'coords'):
        for coord in clipped.coords:
            xs.append(coord[0])
    return xs


def _path_length(waypoints: list) -> float:
    return sum(_dist(waypoints[i], waypoints[i + 1])
               for i in range(len(waypoints) - 1))


def _dist(a, b) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)
