import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import random
import math

class MovingCube(Node):
    def __init__(self):
        super().__init__('moving_cube')
        self.publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.x = -5.0
        self.y = 0.0
        self.z = 0.0
        self.bars = sorted([random.uniform(-4.0, 4.0) for _ in range(5)])
        self.get_logger().info(f"Bars at: {self.bars}")

        self.state = "forward"
        self.speed = 0.05

        # For parabolic jump
        self.jump_origin_x = 0.0
        self.jump_target_x = 0.0
        self.jump_peak_z = 0.6
        self.jump_progress = 0.0
        self.jump_duration = 20  # number of frames in jump

    def timer_callback(self):
        self.update_state()
        self.publish_cube()
        self.publish_pose_arrow()
        self.publish_bars()

    def update_state(self):
        if self.x >= 10.0:
            self.get_logger().info("Reached grid boundary. Resetting cube.")
            self.x = -5.0
            self.z = 0.0
            self.state = "forward"
            self.bars = sorted([random.uniform(-4.0, 4.0) for _ in range(5)])
            return

        bar_ahead = any(abs(bar_x - self.x) < 0.3 for bar_x in self.bars)

        if self.state == "forward":
            if bar_ahead:
                self.state = "jumping"
                self.jump_origin_x = self.x
                self.jump_target_x = self.x + 0.6
                self.jump_progress = 0.0
            else:
                self.x += self.speed

        elif self.state == "jumping":
            self.jump_progress += 1.0 / self.jump_duration
            if self.jump_progress >= 1.0:
                self.jump_progress = 1.0
                self.state = "forward"

            # Interpolate x position
            self.x = (1 - self.jump_progress) * self.jump_origin_x + self.jump_progress * self.jump_target_x

            # Parabola for z
            self.z = 4 * self.jump_peak_z * self.jump_progress * (1 - self.jump_progress)

    def publish_cube(self):
        cube = Marker()
        cube.header.frame_id = "map"
        cube.header.stamp = self.get_clock().now().to_msg()
        cube.ns = "moving_cube"
        cube.id = 0
        cube.type = Marker.CUBE
        cube.action = Marker.ADD
        cube.scale.x = 0.3
        cube.scale.y = 0.3
        cube.scale.z = 0.3
        cube.color.r = 0.1
        cube.color.g = 0.9
        cube.color.b = 0.2
        cube.color.a = 1.0
        cube.pose.position.x = self.x
        cube.pose.position.y = self.y
        cube.pose.position.z = self.z
        cube.pose.orientation.w = 1.0
        self.publisher.publish(cube)

    def publish_pose_arrow(self):
        arrow = Marker()
        arrow.header.frame_id = "map"
        arrow.header.stamp = self.get_clock().now().to_msg()
        arrow.ns = "pose_arrow"
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.scale.x = 0.4
        arrow.scale.y = 0.05
        arrow.scale.z = 0.05
        arrow.color.r = 1.0
        arrow.color.g = 0.2
        arrow.color.b = 0.2
        arrow.color.a = 1.0
        arrow.pose.position.x = self.x
        arrow.pose.position.y = self.y
        arrow.pose.position.z = self.z + 0.3
        arrow.pose.orientation.w = 1.0
        self.publisher.publish(arrow)

    def publish_bars(self):
        bars = Marker()
        bars.header.frame_id = "map"
        bars.header.stamp = self.get_clock().now().to_msg()
        bars.ns = "bars"
        bars.id = 2
        bars.type = Marker.LINE_LIST
        bars.action = Marker.ADD
        bars.scale.x = 0.05
        bars.color.r = 0.2
        bars.color.g = 0.2
        bars.color.b = 1.0
        bars.color.a = 1.0
        bars.lifetime.sec = 0
        bars.points = []

        bar_width_y = 2.0
        bar_height_z = 0.3

        for bar_x in self.bars:
            p1 = Point(x=bar_x, y=-bar_width_y / 2, z=bar_height_z)
            p2 = Point(x=bar_x, y=bar_width_y / 2, z=bar_height_z)
            bars.points.append(p1)
            bars.points.append(p2)

        self.publisher.publish(bars)

def main(args=None):
    rclpy.init(args=args)
    node = MovingCube()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
