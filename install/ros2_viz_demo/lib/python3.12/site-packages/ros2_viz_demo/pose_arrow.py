import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion
import math
import tf_transformations

class PoseArrow(Node):
    def __init__(self):
        super().__init__('pose_arrow')
        self.publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.x = 0.0
        self.y = 0.0
        self.angle = 0.0

    def timer_callback(self):
        q = tf_transformations.quaternion_from_euler(0, 0, self.angle)
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.ARROW
        marker.id = 3
        marker.action = Marker.ADD
        marker.scale.x = 0.5
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.a = 1.0
        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        self.angle += math.pi / 2
        if self.angle >= 2 * math.pi:
            self.angle = 0.0

        self.publisher.publish(marker)

def main():
    rclpy.init()
    node = PoseArrow()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
