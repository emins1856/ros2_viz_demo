import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class CircularPointPublisher(Node):
    def __init__(self):
        super().__init__('circular_point_publisher')
        self.get_logger().info("Node initialized!")
        self.publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.angle = 0.0

    def timer_callback(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "circle"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.color.r = 1.0
        marker.color.a = 1.0

        point = Point()
        point.x = math.cos(self.angle)
        point.y = math.sin(self.angle)
        point.z = 0.0
        self.get_logger().info(f"Published point: x={point.x:.2f}, y={point.y:.2f}, z={point.z:.2f}")
        marker.points.append(point)
        self.publisher.publish(marker)

        

        self.angle += 0.1

def main(args=None):
    rclpy.init(args=args)
    node = CircularPointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
