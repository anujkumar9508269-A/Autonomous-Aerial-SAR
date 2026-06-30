import rclpy
from rclpy.node import Node

import os
import csv
import json
from datetime import datetime

from geometry_msgs.msg import PointStamped

RESULTS_DIR = os.path.expanduser('~/sar_ws/results')


class ResultsLogger(Node):

    def __init__(self):
        super().__init__('results_logger')

        os.makedirs(RESULTS_DIR, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path     = os.path.join(RESULTS_DIR, f'geotags_{timestamp}.csv')
        self.geojson_path = os.path.join(RESULTS_DIR, f'geotags_{timestamp}.geojson')

        self.persons = {}

        self.create_subscription(
            PointStamped,
            '/sar/confirmed_tags',
            self.confirmed_tag_callback,
            10)

        self.get_logger().info('ResultsLogger started.')
        self.get_logger().info(f'Will save to:')
        self.get_logger().info(f'  CSV:     {self.csv_path}')
        self.get_logger().info(f'  GeoJSON: {self.geojson_path}')
        self.get_logger().info('Results will be written on shutdown.')


    def confirmed_tag_callback(self, msg):
        track_id  = msg.header.frame_id
        x         = msg.point.x
        y         = msg.point.y
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if track_id not in self.persons:
            self.persons[track_id] = {
                'x':          x,
                'y':          y,
                'count':      1,
                'first_seen': timestamp
            }
            self.get_logger().info(
                f'New person logged: ID={track_id} '
                f'at ({x:.3f}, {y:.3f})')
        else:
            self.persons[track_id]['x']     = x
            self.persons[track_id]['y']     = y
            self.persons[track_id]['count'] += 1


    def write_results(self):
        if not self.persons:
            self.get_logger().warn('No detections to log.')
            return

        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'track_id', 'x', 'y',
                'observation_count', 'first_seen_sec'
            ])
            for track_id, data in self.persons.items():
                writer.writerow([
                    track_id,
                    round(data['x'], 4),
                    round(data['y'], 4),
                    data['count'],
                    round(data['first_seen'], 3)
                ])

        geojson = {
            'type': 'FeatureCollection',
            'features': [
                {
                    'type': 'Feature',
                    'geometry': {
                        'type': 'Point',
                        'coordinates': [data['x'], data['y'], 0.0]
                    },
                    'properties': {
                        'track_id':          track_id,
                        'observation_count': data['count'],
                        'first_seen_sec':    data['first_seen']
                    }
                }
                for track_id, data in self.persons.items()
            ]
        }
        with open(self.geojson_path, 'w') as f:
            json.dump(geojson, f, indent=2)

        self.get_logger().info(
            f'Results written. {len(self.persons)} person(s) logged.')
        self.get_logger().info(f'  CSV:     {self.csv_path}')
        self.get_logger().info(f'  GeoJSON: {self.geojson_path}')

        self.get_logger().info('Final geo-tag summary:')
        for track_id, data in self.persons.items():
            self.get_logger().info(
                f'  Person ID={track_id}: '
                f'({data["x"]:.3f}, {data["y"]:.3f}) '
                f'from {data["count"]} observations')

    def destroy_node(self):
        self.get_logger().info('Shutting down — writing results to disk...')
        self.write_results()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ResultsLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
