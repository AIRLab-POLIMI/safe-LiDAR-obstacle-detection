#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import csv
import os

class PolygonMarkerPublisher(Node):
    def __init__(self):
        super().__init__('polygon_marker_publisher')
        self.marker_pub = self.create_publisher(Marker, 'polygon_marker_topic', 10)
        self.timer = self.create_timer(1.0, self.publish_markers)

        # Read the CSV file path from the parameter
        self.declare_parameter('csv_file_path', 'known_obs_coord.csv')
        self.csv_file_path = self.get_parameter('csv_file_path').value

        # Read the markers frame_id from the parameter
        self.declare_parameter('markers_frame_id', 'map')
        self.markers_frame_id = self.get_parameter('markers_frame_id').value

    def publish_markers(self):
        marker = Marker()
        marker.header.frame_id = self.markers_frame_id  # Set the frame ID of the polygons
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.2  # Set the scale
        marker.scale.y = 0.2

        # Set the color
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        # Read vertices from the CSV file
        with open(self.csv_file_path, 'r') as csv_file:
            csv_reader = csv.reader(csv_file)
            for row in csv_reader:
                # Check if there are enough coordinates for a point (at least x,y)
                if len(row) >= 2:
                    # Extract x and y coordinates from each pair
                    x_values = [float(row[i]) for i in range(0, len(row), 2)]
                    y_values = [float(row[i+1]) for i in range(0, len(row), 2)]

                    # Create Point messages and add them to the marker
                    for x, y in zip(x_values, y_values):
                        point = Point(x=x, y=y, z=0.0)
                        marker.points.append(point)

        self.marker_pub.publish(marker)

def main():
    rclpy.init()
    node = PolygonMarkerPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

