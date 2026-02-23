import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import random

class MarkerArrayPublisher(Node):
    def __init__(self):
        super().__init__('marker_array_publisher')
        self.publisher = self.create_publisher(MarkerArray, 'marker_array', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = "lines"
        marker.id = 1
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.g = 1.0
        marker.color.a = 1.0

        for i in range(5):
            x = random.uniform(-3, 3)
            start = Point(x=x, y=0.0, z=0.0)
            end = Point(x=x, y=0.0, z=1.0)
            marker.points.append(start)
            marker.points.append(end)

        marker_array.markers.append(marker)
        self.publisher.publish(marker_array)

def main():
    rclpy.init()
    node = MarkerArrayPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
