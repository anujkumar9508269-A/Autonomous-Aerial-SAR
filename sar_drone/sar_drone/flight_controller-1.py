import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
import time


class FlightController(Node):

    def __init__(self):
        super().__init__('flight_controller')

        # Subscribe to drone state
        # tells us if drone is armed and what mode it is in
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10)

        # Subscribe to current drone position
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            10)

        # Publish where we want the drone to go
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10)

        # Service to arm or disarm the drone
        self.arming_client = self.create_client(
            CommandBool,
            '/mavros/cmd/arming')

        # Service to change flight mode
        self.mode_client = self.create_client(
            SetMode,
            '/mavros/set_mode')

        # Store current state and pose
        self.current_state = State()
        self.current_pose  = PoseStamped()
        self.target_pose   = PoseStamped()

        # ArduPilot requires setpoints published
        # at 20Hz before it accepts GUIDED mode
        # This timer handles that automatically
        self.timer = self.create_timer(
            0.05, self.publish_setpoint)

        self.get_logger().info(
            'Flight controller initialised')

    def state_callback(self, msg):
        self.current_state = msg

    def pose_callback(self, msg):
        self.current_pose = msg

    def publish_setpoint(self):
        self.target_pose.header.stamp = \
            self.get_clock().now().to_msg()
        self.target_pose.header.frame_id = 'map'
        self.setpoint_pub.publish(self.target_pose)

    def wait_for_connection(self):
        self.get_logger().info(
            'Waiting for MAVROS connection...')
        while not self.current_state.connected:
            rclpy.spin_once(self)
            time.sleep(0.1)
        self.get_logger().info(
            'MAVROS connected')

    def set_mode(self, mode):
        self.mode_client.wait_for_service(
            timeout_sec=5.0)
        req             = SetMode.Request()
        req.custom_mode = mode
        future          = self.mode_client\
                              .call_async(req)
        rclpy.spin_until_future_complete(
            self, future)
        if future.result().mode_sent:
            self.get_logger().info(
                f'Mode set to {mode}')
        else:
            self.get_logger().warn(
                f'Failed to set mode {mode}')

    def arm(self):
        self.arming_client.wait_for_service(
            timeout_sec=5.0)
        req       = CommandBool.Request()
        req.value = True
        future    = self.arming_client\
                        .call_async(req)
        rclpy.spin_until_future_complete(
            self, future)
        if future.result().success:
            self.get_logger().info('Armed')
        else:
            self.get_logger().warn(
                'Arming failed')

    def takeoff(self, height=10.0):
        self.target_pose.pose.position.x = 0.0
        self.target_pose.pose.position.y = 0.0
        self.target_pose.pose.position.z = height

        self.get_logger().info(
            'Streaming setpoints before arm...')
        start = time.time()
        while time.time() - start < 2.0:
            rclpy.spin_once(self)
            time.sleep(0.05)

        self.set_mode('GUIDED')
        self.arm()

        self.get_logger().info(
            f'Taking off to {height}m...')

        while True:
            rclpy.spin_once(self)
            current_z = \
                self.current_pose.pose.position.z
            if abs(current_z - height) < 0.5:
                break
            time.sleep(0.1)

        self.get_logger().info(
            f'Reached {height}m — hovering')

    def hover(self, duration=5.0):
        self.get_logger().info(
            f'Hovering for {duration}s...')
        start = time.time()
        while time.time() - start < duration:
            rclpy.spin_once(self)
            time.sleep(0.1)
        self.get_logger().info(
            'Hover complete')

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

    fc.wait_for_connection()
    fc.takeoff(height=10.0)
    fc.hover(duration=5.0)
    fc.land()

    fc.get_logger().info('Mission complete')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
