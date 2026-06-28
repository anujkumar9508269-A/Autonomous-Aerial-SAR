"""
coverage_planner/waypoint_follower.py
======================================
ROS 2 node — receives the coverage path and commands the drone via MAVROS.

v3.3 fixes (on top of v3.2):
  [FIX-ARM-RETRY]   ARM state called _transition(S.ARM) immediately on failure,
                    flooding MAVROS with call_async() every 100ms →
                    "Promise already satisfied" crash.
                    Fix: 5-second cooldown tracked via _arm_retry_start before
                    re-calling _transition(S.ARM). Pending future is checked
                    first; cooldown only applies on confirmed failure.

  [FIX-WP0-SKIP]    _along_track_progress(prev=target, target) returns 0.0
                    for WP-0 (prev == target, seg_len < 1e-6 guard returns 0.0).
                    Threshold was >= 0.0, so WP-0 was flagged "advanced"
                    immediately after FOLLOW entry, skipping it entirely.
                    Fix: threshold raised to >= 1.0 (drone must be >= 1m past
                    the waypoint perpendicular). Also guard: only use along-track
                    check when current_idx > 0.

  [FIX-HOLD-ALT]    _hold_position() published _mission_altitude (e.g. 20m)
                    while drone was still on the ground in WAIT_GUIDED /
                    PREARM_CHECK / ARM states. Semantically wrong; uses
                    drone_pose.z (actual height) instead so the hold setpoint
                    stays at ground level until takeoff is commanded.

  [FIX-DISCONN]     Mid-mission MAVROS disconnect (mavros_connected → False)
                    was silently ignored. Added detection in _state_cb: if
                    connection drops after ARM state, log a prominent error.
                    Mission does not auto-abort (ArduPilot continues in GUIDED
                    on its own) but the log makes the issue visible.

  [FIX-GPS-WARN]    When min_gps_fix=0 (default for SITL), the GPS block in
                    PREARM_CHECK was silently skipped, leaving gps_lat/lon=0.0
                    possible at CommandTOL time. Added an explicit log warning
                    if gps_lat==0.0 && gps_lon==0.0 when transitioning to
                    TAKEOFF, so the operator knows lat/lon may be default.

v3.2 fixes (carried forward):
  [FIX-GPS]      GPS fix type calculation was wrong for SITL.
  [FIX-CONN]     State.connected was never read from /mavros/state.
  [FIX-STREAM]   No setpoints published during WAIT_GUIDED / PREARM_CHECK /
                 ARM states.
  [FIX-TOL]      CommandTOL only set altitude, not lat/lon.
  [FIX-ALTCHK]   TAKEOFF transitioned to FOLLOW after fixed 6s timer only.
  [FIX-GUIDED]   guided_timeout_s raised to 30s.

State machine:
  IDLE        -> path received               -> WAIT_GUIDED
  WAIT_GUIDED -> mavros connected + GUIDED   -> PREARM_CHECK
  PREARM_CHECK-> GPS fix OK                  -> ARM
  ARM         -> arm confirmed               -> TAKEOFF
  TAKEOFF     -> altitude reached + timer    -> FOLLOW
  FOLLOW      -> all waypoints reached       -> LAND
  LAND        -> (terminal)
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix, BatteryState


# ---------------------------------------------------------------------------
# State constants
# ---------------------------------------------------------------------------

class S:
    IDLE         = 'IDLE'
    WAIT_GUIDED  = 'WAIT_GUIDED'
    PREARM_CHECK = 'PREARM_CHECK'
    ARM          = 'ARM'
    TAKEOFF      = 'TAKEOFF'
    FOLLOW       = 'FOLLOW'
    LAND         = 'LAND'


class WaypointFollower(Node):

    def __init__(self):
        super().__init__('waypoint_follower')

        # ---- Parameters ------------------------------------------------
        self.declare_parameter('acceptance_radius',  2.0)
        self.declare_parameter('takeoff_wait_s',     8.0)    # min climb time
        self.declare_parameter('guided_timeout_s',  30.0)    # [FIX-GUIDED] was 10
        self.declare_parameter('cte_gain',           0.5)
        self.declare_parameter('min_gps_fix_type',   0)      # [FIX-GPS] 0=any fix
        self.declare_parameter('min_battery_pct',    0.0)    # 0 = disabled in SITL
        self.declare_parameter('wp_timeout_s',      45.0)    # longer for SITL
        self.declare_parameter('arm_retry_cooldown', 5.0)    # [FIX-ARM-RETRY]

        self.accept_r         = self.get_parameter('acceptance_radius').value
        self.takeoff_wait     = self.get_parameter('takeoff_wait_s').value
        self.guided_timeout   = self.get_parameter('guided_timeout_s').value
        self.cte_gain         = self.get_parameter('cte_gain').value
        self.min_gps_fix      = self.get_parameter('min_gps_fix_type').value
        self.min_battery      = self.get_parameter('min_battery_pct').value
        self.wp_timeout       = self.get_parameter('wp_timeout_s').value
        self.arm_retry_cd     = self.get_parameter('arm_retry_cooldown').value

        # ---- Mission state ---------------------------------------------
        self.state_machine      = S.IDLE
        self.waypoints          = []
        self.current_idx        = 0
        self.drone_pose         = None
        self.armed              = False
        self.mode               = ''
        self.mavros_connected   = False    # [FIX-CONN]
        self._prev_connected    = False    # [FIX-DISCONN] track changes
        self._land_called       = False
        self._pending_future: Future = None
        self._state_timer_start = None
        self._wp_timer_start    = None
        self._arm_retry_start   = None    # [FIX-ARM-RETRY] cooldown timer

        # Telemetry state
        self.gps_fix_ok         = False    # [FIX-GPS] simple bool
        self.gps_lat            = 0.0      # [FIX-TOL] for CommandTOL lat/lon
        self.gps_lon            = 0.0
        self.battery_pct        = 100.0
        self._mission_altitude  = 20.0     # fallback, overwritten from path

        # ---- QoS — MUST match MAVROS publisher (BEST_EFFORT) -----------
        _mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ---- Subscribers -----------------------------------------------
        self.create_subscription(
            Path,        '/coverage_path',
            self._path_cb, 10                # our publisher — RELIABLE
        )
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self._pose_cb, _mavros_qos       # MAVROS → BEST_EFFORT
        )
        self.create_subscription(
            State,       '/mavros/state',
            self._state_cb, _mavros_qos      # MAVROS → BEST_EFFORT
        )
        self.create_subscription(
            NavSatFix,    '/mavros/global_position/global',
            self._gps_cb, _mavros_qos        # MAVROS → BEST_EFFORT
        )
        self.create_subscription(
            BatteryState, '/mavros/battery',
            self._battery_cb, _mavros_qos    # MAVROS → BEST_EFFORT
        )

        # ---- Publisher -------------------------------------------------
        self.sp_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10
        )

        # ---- Service clients -------------------------------------------
        self.set_mode_client = self.create_client(SetMode,     '/mavros/set_mode')
        self.arming_client   = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_client  = self.create_client(CommandTOL,  '/mavros/cmd/takeoff')

        for name, client in [
            ('set_mode', self.set_mode_client),
            ('arming',   self.arming_client),
            ('takeoff',  self.takeoff_client),
        ]:
            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(
                    f'{name} service not yet available — will retry at mission start'
                )

        # ---- 10 Hz control loop ----------------------------------------
        self.create_timer(0.1, self._control_loop)
        self.get_logger().info('Waypoint follower ready — waiting for path and MAVROS...')

    # ------------------------------------------------------------------
    # Subscribers
    # ------------------------------------------------------------------

    def _path_cb(self, msg: Path):
        if self.waypoints:
            return
        self.waypoints   = msg.poses
        self.current_idx = 0
        if self.waypoints:
            alt = self.waypoints[0].pose.position.z
            if alt > 1.0:
                self._mission_altitude = alt
        self.get_logger().info(
            f'Path received: {len(self.waypoints)} waypoints  '
            f'altitude={self._mission_altitude:.1f}m'
        )
        self._transition(S.WAIT_GUIDED)

    def _pose_cb(self, msg: PoseStamped):
        self.drone_pose = msg.pose.position

    def _state_cb(self, msg: State):
        self.armed            = msg.armed
        self.mode             = msg.mode
        self._prev_connected  = self.mavros_connected
        self.mavros_connected = msg.connected    # [FIX-CONN]

        # [FIX-DISCONN] Warn loudly if connection drops mid-mission
        if self._prev_connected and not self.mavros_connected:
            self.get_logger().error(
                'MAVROS FCU connection LOST! '
                f'Current state: {self.state_machine}. '
                'ArduPilot will continue autonomously in GUIDED. '
                'Check SITL/MAVROS processes.'
            )

    def _gps_cb(self, msg: NavSatFix):
        # [FIX-GPS] NavSatStatus.status: -1=no fix, 0=fix, 1=SBAS, 2=GBAS
        # Any value >= 0 means we have a usable fix.
        self.gps_fix_ok = (msg.status.status >= 0)
        self.gps_lat    = msg.latitude     # [FIX-TOL]
        self.gps_lon    = msg.longitude

    def _battery_cb(self, msg: BatteryState):
        if msg.percentage >= 0:
            self.battery_pct = msg.percentage * 100.0

    # ------------------------------------------------------------------
    # State machine
    # ------------------------------------------------------------------

    def _transition(self, new_state: str):
        self.get_logger().info(f'State: {self.state_machine} -> {new_state}')
        self.state_machine      = new_state
        self._state_timer_start = self.get_clock().now()
        self._pending_future    = None

        if new_state == S.WAIT_GUIDED:
            # Only request GUIDED if already connected; otherwise control
            # loop will request it once connected. [FIX-CONN]
            if self.mavros_connected:
                self._send_set_mode('GUIDED')
            else:
                self.get_logger().info(
                    'MAVROS not yet connected — will request GUIDED once connected'
                )

        elif new_state == S.PREARM_CHECK:
            self.get_logger().info(
                f'Pre-arm check: gps_fix_ok={self.gps_fix_ok}  '
                f'battery={self.battery_pct:.0f}%'
            )

        elif new_state == S.ARM:
            # [FIX-ARM-RETRY] Reset cooldown timer on every fresh ARM entry
            self._arm_retry_start = self.get_clock().now()
            self._send_arm()

        elif new_state == S.TAKEOFF:
            # [FIX-GPS-WARN] Warn if GPS lat/lon are still 0.0 default
            if self.gps_lat == 0.0 and self.gps_lon == 0.0:
                self.get_logger().warn(
                    'CommandTOL: gps_lat and gps_lon are 0.0 — GPS fix may not '
                    'have been received yet. CommandTOL may be sent with '
                    'lat=0 lon=0. SITL fallback setpoint will handle this.'
                )
            # [FIX-TOL] Include lat/lon for ArduPilot SITL CommandTOL
            alt              = self._mission_altitude
            to_req           = CommandTOL.Request()
            to_req.altitude  = float(alt)
            to_req.latitude  = float(self.gps_lat)    # [FIX-TOL]
            to_req.longitude = float(self.gps_lon)    # [FIX-TOL]
            to_req.min_pitch = 0.0
            to_req.yaw       = 0.0
            self._pending_future = self.takeoff_client.call_async(to_req)
            self.get_logger().info(
                f'Takeoff to {alt:.1f}m  '
                f'lat={self.gps_lat:.6f}  lon={self.gps_lon:.6f}'
            )

        elif new_state == S.LAND:
            self._do_land()

    def _send_arm(self):
        """Send a single arming request. Called from _transition and retry."""
        if not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('arming service unavailable — will retry after cooldown')
            return
        req       = CommandBool.Request()
        req.value = True
        self._pending_future = self.arming_client.call_async(req)
        self.get_logger().info('Sending arm command...')

    def _send_set_mode(self, custom_mode: str):
        if not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('set_mode service unavailable — will retry')
            return
        req             = SetMode.Request()
        req.custom_mode = custom_mode
        self._pending_future = self.set_mode_client.call_async(req)
        self.get_logger().info(f'Requesting mode: {custom_mode}')

    # ------------------------------------------------------------------
    # [FIX-STREAM] Setpoint hold — call in every pre-flight state
    # ------------------------------------------------------------------

    def _hold_position(self):
        """
        Publish current position as setpoint to maintain ArduPilot's required
        >=2 Hz setpoint stream. Without this, ArduPilot disarms the drone
        seconds after arming due to setpoint stream timeout.

        [FIX-HOLD-ALT] Uses drone_pose.z (actual height) rather than
        _mission_altitude so the hold setpoint stays at ground level before
        takeoff is commanded.
        """
        if self.drone_pose is None:
            return
        self._publish_setpoint_xyz(
            self.drone_pose.x,
            self.drone_pose.y,
            self.drone_pose.z,    # [FIX-HOLD-ALT] actual height, not mission alt
        )

    # ------------------------------------------------------------------
    # 10 Hz control loop
    # ------------------------------------------------------------------

    def _control_loop(self):
        now = self.get_clock().now()

        if self.state_machine == S.WAIT_GUIDED:
            # [FIX-CONN] Do not request GUIDED until connected
            if not self.mavros_connected:
                if (now.nanoseconds // 5_000_000_000) % 2 == 0:
                    self.get_logger().info('Waiting for MAVROS FCU connection...')
                return

            self._hold_position()    # [FIX-STREAM]

            if self.mode == 'GUIDED':
                self._transition(S.PREARM_CHECK)
                return

            elapsed = (now - self._state_timer_start).nanoseconds * 1e-9
            if elapsed > self.guided_timeout:
                self.get_logger().warn(
                    f'Still waiting for GUIDED mode ({elapsed:.0f}s). '
                    f'Current mode: "{self.mode}". '
                    f'EKF may still be initialising — this is normal in SITL. '
                    f'Retrying set_mode...'
                )
                self._state_timer_start = now
                self._send_set_mode('GUIDED')

        elif self.state_machine == S.PREARM_CHECK:
            self._hold_position()    # [FIX-STREAM]

            # [FIX-GPS] check gps_fix_ok bool, not integer comparison.
            # If min_gps_fix=0 (SITL default), skip GPS check entirely.
            if self.min_gps_fix > 0 and not self.gps_fix_ok:
                elapsed = (now - self._state_timer_start).nanoseconds * 1e-9
                if int(elapsed) % 5 == 0:
                    self.get_logger().warn(
                        f'Waiting for GPS fix (gps_fix_ok={self.gps_fix_ok}, '
                        f'lat={self.gps_lat:.4f})...'
                    )
                return

            if self.min_battery > 0 and self.battery_pct < self.min_battery:
                self.get_logger().error(
                    f'Battery {self.battery_pct:.0f}% below min '
                    f'{self.min_battery:.0f}% — mission aborted'
                )
                return

            self.get_logger().info(
                f'Pre-arm OK: gps_fix={self.gps_fix_ok}  '
                f'battery={self.battery_pct:.0f}%'
            )
            self._transition(S.ARM)

        elif self.state_machine == S.ARM:
            self._hold_position()    # [FIX-STREAM] critical — keep stream alive

            if self._pending_future is not None and self._pending_future.done():
                result = self._pending_future.result()
                if result is not None and result.success:
                    self.get_logger().info('Armed successfully')
                    self._transition(S.TAKEOFF)
                else:
                    # [FIX-ARM-RETRY] Enforce cooldown before retrying.
                    # Do NOT call _transition(S.ARM) immediately — that floods
                    # MAVROS with call_async() every 100ms → "Promise already
                    # satisfied" crash.
                    elapsed_since_fail = (
                        (now - self._arm_retry_start).nanoseconds * 1e-9
                        if self._arm_retry_start is not None else self.arm_retry_cd
                    )
                    if elapsed_since_fail < self.arm_retry_cd:
                        # Still in cooldown — keep publishing setpoints, wait
                        return
                    self.get_logger().error(
                        f'Arming failed — retrying after {self.arm_retry_cd:.0f}s cooldown...'
                    )
                    # Reset cooldown timer and send a fresh arm request
                    self._arm_retry_start = now
                    self._pending_future  = None
                    self._send_arm()
            # If future is still pending (in-flight), do nothing — just hold.

        elif self.state_machine == S.TAKEOFF:
            if self._pending_future is not None and self._pending_future.done():
                result = self._pending_future.result()
                if result is None or not result.success:
                    self.get_logger().error('Takeoff command failed — retrying...')
                    self._transition(S.TAKEOFF)
                    return

            elapsed = (now - self._state_timer_start).nanoseconds * 1e-9

            # [FIX-ALTCHK] Check both timer AND actual altitude
            target_alt  = self._mission_altitude
            current_alt = self.drone_pose.z if self.drone_pose else 0.0
            alt_reached = current_alt >= target_alt * 0.85

            if elapsed >= self.takeoff_wait and alt_reached:
                self.get_logger().info(
                    f'Climb complete — alt={current_alt:.1f}m '
                    f'target={target_alt:.1f}m — starting waypoint follow'
                )
                self._transition(S.FOLLOW)
            elif elapsed >= self.takeoff_wait * 3:
                # Safety: if 3x timer elapsed and altitude still not reached,
                # log warning and proceed anyway (avoid hanging forever)
                self.get_logger().warn(
                    f'Takeoff timeout — alt={current_alt:.1f}m '
                    f'(expected {target_alt:.1f}m) — proceeding anyway'
                )
                self._transition(S.FOLLOW)
            '''else:
                # Publish a purely vertical climb setpoint to keep the stream alive
                # without commanding lateral movement that cancels CommandTOL
                if self.drone_pose:
                    self._publish_setpoint_xyz(
                        self.drone_pose.x, 
                        self.drone_pose.y, 
                        target_alt
                    )'''
                  
     #---------------------------------------------------------------------------
                    
            #else:
                # Publish climb setpoint
             #   if self.waypoints:
              #      self._publish_setpoint(self.waypoints[0].pose.position)

        elif self.state_machine == S.FOLLOW:
            if not self.waypoints or self.drone_pose is None:
                return
            if self.current_idx >= len(self.waypoints):
                self._transition(S.LAND)
                return

            target = self.waypoints[self.current_idx].pose.position
            prev   = (self.waypoints[self.current_idx - 1].pose.position
                      if self.current_idx > 0 else target)

            # CTE correction
            commanded = self._cte_corrected_target(target)
            self._publish_setpoint_xyz(commanded[0], commanded[1], target.z)

            dist     = self._dist3(self.drone_pose, target)
            advanced = False

            if dist < self.accept_r:
                # Primary: within acceptance radius sphere
                advanced = True
            elif self.current_idx > 0 and self._along_track_progress(prev, target) >= 1.0:
                # [FIX-WP0-SKIP] Secondary: drone has passed >= 1m beyond the
                # waypoint perpendicular. Guard current_idx > 0 so WP-0 is
                # never skipped via this path (prev==target → seg_len<1e-6 →
                # returns 0.0, which was >= 0.0 in v3.2 — instant WP-0 skip).
                advanced = True

            if advanced:
                self.get_logger().info(
                    f'WP {self.current_idx + 1}/{len(self.waypoints)} '
                    f'reached  dist={dist:.2f}m'
                )
                self.current_idx    += 1
                self._wp_timer_start = None
                return

            # Stall timeout
            if self._wp_timer_start is None:
                self._wp_timer_start = now
            elapsed_at_wp = (now - self._wp_timer_start).nanoseconds * 1e-9
            if elapsed_at_wp > self.wp_timeout:
                self.get_logger().warn(
                    f'WP {self.current_idx + 1}/{len(self.waypoints)} '
                    f'timed out after {self.wp_timeout:.0f}s '
                    f'(dist={dist:.2f}m) — forcing advance'
                )
                self.current_idx    += 1
                self._wp_timer_start = None

        elif self.state_machine == S.LAND:
            pass

    # ------------------------------------------------------------------
    # Along-track progress check
    # ------------------------------------------------------------------

    def _along_track_progress(self, prev, target) -> float:
        """
        Returns the signed distance (metres) of the drone past the target
        waypoint along the prev→target track direction.
        Positive = drone has passed the waypoint.
        Returns 0.0 if segment is degenerate (prev == target).
        """
        dx = target.x - prev.x
        dy = target.y - prev.y
        seg_len = math.sqrt(dx ** 2 + dy ** 2)
        if seg_len < 1e-6:
            return 0.0
        ux, uy = dx / seg_len, dy / seg_len
        to_drone_x = self.drone_pose.x - target.x
        to_drone_y = self.drone_pose.y - target.y
        return to_drone_x * ux + to_drone_y * uy

    # ------------------------------------------------------------------
    # CTE controller
    # ------------------------------------------------------------------

    def _cte_corrected_target(self, target) -> tuple:
        if self.current_idx == 0 or self.drone_pose is None:
            return (target.x, target.y)
        prev    = self.waypoints[self.current_idx - 1].pose.position
        dx      = target.x - prev.x
        dy      = target.y - prev.y
        seg_len = math.sqrt(dx ** 2 + dy ** 2)
        if seg_len < 1e-6:
            return (target.x, target.y)
        ux, uy = dx / seg_len, dy / seg_len
        px, py = -uy, ux
        to_drone_x = self.drone_pose.x - prev.x
        to_drone_y = self.drone_pose.y - prev.y
        cte        = to_drone_x * px + to_drone_y * py
        return (
            target.x - self.cte_gain * cte * px,
            target.y - self.cte_gain * cte * py,
        )

    # ------------------------------------------------------------------
    # Utilities
    # ------------------------------------------------------------------

    def _publish_setpoint(self, position):
        self._publish_setpoint_xyz(position.x, position.y, position.z)

    def _publish_setpoint_xyz(self, x: float, y: float, z: float):
        sp                    = PoseStamped()
        sp.header.stamp       = self.get_clock().now().to_msg()
        sp.header.frame_id    = 'map'
        sp.pose.position.x    = float(x)
        sp.pose.position.y    = float(y)
        sp.pose.position.z    = float(z)
        sp.pose.orientation.w = 1.0
        self.sp_pub.publish(sp)

    def _dist3(self, a, b) -> float:
        return math.sqrt(
            (a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2
        )

    def _do_land(self):
        if self._land_called:
            return
        self._land_called = True
        self.get_logger().info('Mission complete — switching to LAND mode')
        self._send_set_mode('LAND')


# ---------------------------------------------------------------------------

def main():
    rclpy.init()
    node = WaypointFollower()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
