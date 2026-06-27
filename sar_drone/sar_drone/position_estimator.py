import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped


class PositionEstimator(Node):

    def __init__(self):
        super().__init__('position_estimator')
      
        self.estimates = {}

        self.create_subscription(
            PointStamped,
            '/sar/raw_tags',
            self.raw_tag_callback,
            10)

        self.confirmed_pub = self.create_publisher(
            PointStamped, '/sar/confirmed_tags', 10)

        self.get_logger().info('PositionEstimator started. Waiting for raw tags...')

    def raw_tag_callback(self, msg):
        track_id = msg.header.frame_id
        new_x    = msg.point.x
        new_y    = msg.point.y

        if track_id not in self.estimates:
            # First observation for this person — initialise
            self.estimates[track_id] = {
                'x':     new_x,
                'y':     new_y,
                'count': 1
            }
            self.get_logger().info(
                f'New person detected! ID={track_id} '
                f'first estimate: ({new_x:.3f}, {new_y:.3f})')
        else:
            # Update running mean
            est   = self.estimates[track_id]
            count = est['count'] + 1
            est['x']     = est['x'] + (new_x - est['x']) / count
            est['y']     = est['y'] + (new_y - est['y']) / count
            est['count'] = count

            self.get_logger().info(
                f'ID={track_id} updated estimate: '
                f'({est["x"]:.3f}, {est["y"]:.3f}) '
                f'from {count} observations')

        self.publish_estimate(track_id)

    def publish_estimate(self, track_id):
        est = self.estimates[track_id]

        msg = PointStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = track_id
        msg.point.x = est['x']
        msg.point.y = est['y']
        msg.point.z = 0.0
        self.confirmed_pub.publish(msg)

    def destroy_node(self):
        self.get_logger().info('Shutting down. Final estimates:')
        for track_id, est in self.estimates.items():
            self.get_logger().info(
                f'  Person ID={track_id}: '
                f'({est["x"]:.3f}, {est["y"]:.3f}) '
                f'from {est["count"]} observations')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PositionEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
