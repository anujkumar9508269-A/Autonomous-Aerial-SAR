import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
import time


class FlightController(Node):

    def __init__(self):
        super().__init__('flight_controller')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10)

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos)

        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_callback,
            qos)

        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10)

        self.arming_client = self.create_client(
            CommandBool,
            '/mavros/cmd/arming')

        self.mode_client = self.create_client(
            SetMode,
            '/mavros/set_mode')

        self.takeoff_client = self.create_client(
            CommandTOL,
            '/mavros/cmd/takeoff')

        self.current_state = State()
        self.current_pose  = PoseStamped()
        self.target_pose   = PoseStamped()
        self.current_gps   = None

        self.timer = self.create_timer(
            0.05, self.publish_setpoint)

        self.get_logger().info(
            'Flight controller initialised')

    def state_callback(self, msg):
        self.current_state = msg

    def pose_callback(self, msg):
        self.current_pose = msg

    def gps_callback(self, msg):
        self.current_gps = msg

    def publish_setpoint(self):
        self.target_pose.header.stamp = \
            self.get_clock().now().to_msg()
        self.target_pose.header.frame_id = 'map'
        self.setpoint_pub.publish(self.target_pose)

    def stop_setpoint_timer(self):
        self.timer.cancel()
        self.get_logger().info(
            'Setpoint timer stopped')

    def start_setpoint_timer(self):
        self.timer = self.create_timer(
            0.05, self.publish_setpoint)
        self.get_logger().info(
            'Setpoint timer started')

    def wait_for_connection(self):
        self.get_logger().info(
            'Waiting for MAVROS connection...')
        while not self.current_state.connected:
            rclpy.spin_once(self)
            time.sleep(0.1)
        self.get_logger().info(
            'MAVROS connected')

    def wait_for_gps(self):
        self.get_logger().info(
            'Waiting for GPS fix...')
        timeout = time.time() + 60.0
        while self.current_gps is None:
            rclpy.spin_once(self)
            time.sleep(0.1)
            if time.time() > timeout:
                self.get_logger().warn(
                    'GPS timeout — continuing anyway')
                break
        if self.current_gps is not None:
            self.get_logger().info(
                f'GPS fix: '
                f'lat={self.current_gps.latitude:.6f} '
                f'lon={self.current_gps.longitude:.6f}')

    def set_mode(self, mode):
        self.mode_client.wait_for_service(
            timeout_sec=5.0)
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().mode_sent:
            self.get_logger().info(
                f'Mode set to {mode}')
        else:
            self.get_logger().warn(
                f'Failed to set mode {mode}')

    def arm(self):
        self.arming_client.wait_for_service(
            timeout_sec=5.0)
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info('Armed')
        else:
            self.get_logger().warn('Arming failed')

    def takeoff_command(self, height=10.0):
        # Stop the setpoint timer so it does not
        # interfere with ArduPilot takeoff sequence
        self.stop_setpoint_timer()

        self.takeoff_client.wait_for_service(
            timeout_sec=5.0)

        req           = CommandTOL.Request()
        req.altitude  = height
        req.min_pitch = 0.0
        req.yaw       = 0.0
        req.latitude  = 0.0
        req.longitude = 0.0

        self.get_logger().info(
            f'Sending takeoff to {height}m...')

        future = self.takeoff_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info(
                'Takeoff command accepted')
        else:
            self.get_logger().warn(
                'Takeoff command rejected')
            self.start_setpoint_timer()
            return

        # Wait for altitude
        # Do NOT publish setpoints during this phase
        timeout = time.time() + 30.0
        while time.time() < timeout:
            rclpy.spin_once(self)
            current_z = \
                self.current_pose.pose.position.z
            self.get_logger().info(
                f'Altitude: {current_z:.2f}m',
                throttle_duration_sec=1.0)
            if current_z > height - 0.5:
                self.get_logger().info(
                    f'Reached {height}m')
                break
            if not self.current_state.armed:
                self.get_logger().error(
                    'Drone disarmed during takeoff')
                break
            time.sleep(0.1)

        # Restart timer for position control
        self.start_setpoint_timer()

    def hover(self, duration=5.0):
        self.get_logger().info(
            f'Hovering for {duration}s...')
        self.target_pose.pose.position.x = \
            self.current_pose.pose.position.x
        self.target_pose.pose.position.y = \
            self.current_pose.pose.position.y
        self.target_pose.pose.position.z = \
            self.current_pose.pose.position.z
        start = time.time()
        while time.time() - start < duration:
            rclpy.spin_once(self)
            time.sleep(0.1)
        self.get_logger().info('Hover complete')

    def land(self):
        self.set_mode('LAND')
        self.get_logger().info('Landing...')
        while True:
            rclpy.spin_once(self)
            current_z = \
                self.current_pose.pose.position.z
            if current_z < 0.3:
                break
            time.sleep(0.1)
        self.get_logger().info('Landed')


def main(args=None):
    rclpy.init(args=args)
    fc = FlightController()

    # Wait for MAVROS connection
    fc.wait_for_connection()

    # Wait for GPS fix
    fc.wait_for_gps()

    # Stream setpoints briefly at z=0
    # so ArduPilot does not see a jump command
    fc.get_logger().info(
        'Streaming setpoints...')
    fc.target_pose.pose.position.x = 0.0
    fc.target_pose.pose.position.y = 0.0
    fc.target_pose.pose.position.z = 0.0
    start = time.time()
    while time.time() - start < 3.0:
        rclpy.spin_once(fc)
        time.sleep(0.05)

    # Stop timer before mode change and arming
    fc.stop_setpoint_timer()

    # Set GUIDED mode
    fc.set_mode('GUIDED')
    time.sleep(2.0)
    rclpy.spin_once(fc)

    # Arm with retries
    for attempt in range(10):
        fc.arm()
        time.sleep(0.5)
        rclpy.spin_once(fc)
        if fc.current_state.armed:
            fc.get_logger().info(
                'Armed confirmed')
            break
        fc.get_logger().warn(
            f'Not armed, attempt {attempt + 1}')

    if not fc.current_state.armed:
        fc.get_logger().error(
            'Failed to arm — aborting')
        rclpy.shutdown()
        return

    # Takeoff — timer already stopped
    # timer restarts inside takeoff_command
    # after altitude is reached
    fc.takeoff_command(height=10.0)

    # Hover for 5 seconds
    fc.hover(duration=5.0)

    # Land
    fc.land()

    fc.get_logger().info('Mission complete')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
