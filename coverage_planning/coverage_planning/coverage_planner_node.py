"""
coverage_planner/coverage_planner_node.py
==========================================
ROS 2 node — publishes the optimised coverage path as nav_msgs/Path.

v3.0 fixes:
  - Blocking time.sleep() replaced with one-shot timer (retained from v2).
  - Polygon loaded from YAML (retained from v2).
  - GPS polygon support retained.
  - Live parameter callback retained.
  - NEW: vfov_deg parameter added so strip footprint is accurate for
    non-square sensors.
  - NEW: strategy parameter selects lawnmower / nonconvex / spiral.
  - NEW: Startup log prints computed coverage % using grid-point sampling.
  - NEW: origin_lat / origin_lon declared as node parameters (not only
    in YAML) so they can be overridden from the command line.
"""

import os
import yaml
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from coverage_planner.planner import (
    CoveragePlanner,
    gps_to_local,
    compute_coverage_pct,
    max_safe_altitude,
)


class CoveragePlannerNode(Node):

    def __init__(self):
        super().__init__('coverage_planner_node')

        # ---- Parameters ------------------------------------------------
        self.declare_parameter('altitude',    20.0)
        self.declare_parameter('hfov_deg',    60.0)
        self.declare_parameter('vfov_deg',    60.0)   # NEW: non-square sensor support
        self.declare_parameter('overlap',      0.1)
        # strategy: 'lawnmower' | 'nonconvex' | 'spiral'
        self.declare_parameter('strategy',   'lawnmower')
        self.declare_parameter('polygon_file', '')
        # Optional GPS origin override (can also live in the YAML)
        self.declare_parameter('origin_lat',   0.0)
        self.declare_parameter('origin_lon',   0.0)
        # [v3.2] Optional detection-model-derived pixel threshold. Default
        # (0) disables the check entirely — this node does NOT auto-clamp
        # altitude on your behalf. Set this to the min_px value you derived
        # from recall_vs_altitude_to_min_px() (see README) on YOUR actual
        # YOLOv8 model to get a startup warning if `altitude` exceeds what
        # your model can reliably detect at.
        self.declare_parameter('detection_min_px', 0)

        # ---- Publisher -------------------------------------------------
        self.path_pub = self.create_publisher(Path, '/coverage_path', 10)

        # ---- State -----------------------------------------------------
        self._path_msg      = None
        self._polygon_cache = None

        # ---- Timers ----------------------------------------------------
        self.create_timer(1.0, self._publish_loop)

        # ---- Live param callback ---------------------------------------
        self.add_on_set_parameters_callback(self._on_params_changed)

        self.get_logger().info('Coverage planner node started — planning in 1 s…')

    # ------------------------------------------------------------------
    # Publish loop
    # ------------------------------------------------------------------

    def _publish_loop(self):
        if self._path_msg is None:
            self._path_msg = self._plan()
            if self._path_msg is None:
                return
        self.path_pub.publish(self._path_msg)

    # ------------------------------------------------------------------
    # Planning
    # ------------------------------------------------------------------

    def _plan(self) -> Path:
        alt            = self.get_parameter('altitude').value
        hfov           = self.get_parameter('hfov_deg').value
        vfov           = self.get_parameter('vfov_deg').value
        overlap        = self.get_parameter('overlap').value
        strategy       = self.get_parameter('strategy').value
        poly_file      = self.get_parameter('polygon_file').value
        detection_min_px = self.get_parameter('detection_min_px').value

        # [v3.2] Optional detection-model altitude sanity check. Disabled by
        # default (detection_min_px=0). If you've measured your YOLOv8
        # model's actual recall-vs-altitude behaviour (see
        # recall_vs_altitude_to_min_px() and the README), set
        # detection_min_px to that derived value so a misconfigured
        # `altitude` that exceeds your model's reliable detection range
        # gets flagged here, before the drone ever takes off — instead of
        # silently flying a mission your model can't actually see anything
        # useful in.
        if detection_min_px and detection_min_px > 0:
            ceiling = max_safe_altitude(vfov_deg=vfov, min_px=detection_min_px)
            if alt > ceiling:
                self.get_logger().warn(
                    f'Configured altitude={alt:.1f}m EXCEEDS the detection '
                    f'ceiling of {ceiling:.1f}m derived from '
                    f'detection_min_px={detection_min_px} (your measured '
                    f'model behaviour). Flying this mission may produce '
                    f'low recall — consider lowering altitude or '
                    f're-measuring your model at this altitude first.'
                )

        polygon = self._load_polygon(poly_file)
        if polygon is None:
            return None

        planner = CoveragePlanner(alt, hfov, vfov, overlap)

        if strategy == 'nonconvex':
            waypoints = planner.generate_path_nonconvex(polygon)
        elif strategy == 'spiral':
            waypoints = planner.generate_spiral_path(polygon)
        else:
            waypoints = planner.generate_path(polygon)

        # Grid-point coverage check [FIX-3]
        footprints  = planner.get_detection_footprints(waypoints)
        coverage_pct = compute_coverage_pct(footprints, polygon, grid_step=1.0)

        self.get_logger().info(
            f'Planner: strategy={strategy}  altitude={alt} m  '
            f'strip_width={planner.strip_width:.1f} m  '
            f'safe_width={planner.safe_width:.1f} m  '
            f'waypoints={len(waypoints)}  '
            f'coverage={coverage_pct:.1f}%'
        )

        if coverage_pct < 95.0:
            self.get_logger().warn(
                f'Coverage {coverage_pct:.1f}% is below 95% threshold! '
                f'Consider reducing altitude or overlap.'
            )

        path_msg = self._build_path_msg(waypoints, alt)
        return path_msg

    # ------------------------------------------------------------------
    # Polygon loading
    # ------------------------------------------------------------------

    def _load_polygon(self, polygon_file: str):
        if polygon_file and os.path.isfile(polygon_file):
            with open(polygon_file, 'r') as f:
                data = yaml.safe_load(f)

            if 'gps_polygon' in data:
                origin_lat = data.get('origin_lat',
                             self.get_parameter('origin_lat').value)
                origin_lon = data.get('origin_lon',
                             self.get_parameter('origin_lon').value)
                if not origin_lat or not origin_lon:
                    self.get_logger().error(
                        'GPS polygon needs origin_lat / origin_lon'
                    )
                    return None
                gps_poly = [tuple(p) for p in data['gps_polygon']]
                local    = gps_to_local(gps_poly, origin_lat, origin_lon)
                self.get_logger().info(
                    f'Loaded GPS polygon ({len(local)} vertices), '
                    f'origin=({origin_lat:.6f}, {origin_lon:.6f})'
                )
                return local

            elif 'polygon' in data:
                poly = [tuple(p) for p in data['polygon']]
                self.get_logger().info(
                    f'Loaded local polygon ({len(poly)} vertices)'
                )
                return poly

            else:
                self.get_logger().error(
                    'YAML has neither "polygon" nor "gps_polygon" key'
                )
                return None

        elif polygon_file:
            self.get_logger().error(f'polygon_file not found: {polygon_file}')
            return None

        else:
            self.get_logger().warn(
                'No polygon_file set. Using demo L-shape. '
                'Pass polygon_file:=<path>.yaml for real missions.'
            )
            return [
                (0,   0),
                (100, 0),
                (100, 80),
                (60,  80),
                (60,  40),
                (0,   40),
            ]

    # ------------------------------------------------------------------
    # Live parameter callback
    # ------------------------------------------------------------------

    def _on_params_changed(self, params):
        from rcl_interfaces.msg import SetParametersResult
        triggers = {
            'altitude', 'hfov_deg', 'vfov_deg', 'overlap',
            'strategy', 'polygon_file', 'origin_lat', 'origin_lon',
            'detection_min_px'
        }
        if any(p.name in triggers for p in params):
            self._path_msg = None
            self.get_logger().info('Parameter changed — replanning on next tick')
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # Message builder
    # ------------------------------------------------------------------

    def _build_path_msg(self, waypoints: list, altitude: float) -> Path:
        path                 = Path()
        path.header.frame_id = 'map'
        path.header.stamp    = self.get_clock().now().to_msg()

        for x, y in waypoints:
            pose                     = PoseStamped()
            pose.header.frame_id     = 'map'
            pose.header.stamp        = path.header.stamp
            pose.pose.position.x     = float(x)
            pose.pose.position.y     = float(y)
            pose.pose.position.z     = float(altitude)
            pose.pose.orientation.w  = 1.0
            path.poses.append(pose)

        return path


# ---------------------------------------------------------------------------

def main():
    rclpy.init()
    node = CoveragePlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
