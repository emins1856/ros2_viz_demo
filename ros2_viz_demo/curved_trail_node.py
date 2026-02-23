import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import random

class MovingCubeWithTrail(Node):
    def __init__(self):
        super().__init__('moving_cube_with_trail')
        self.publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.x = -5.0
        self.y = 0.0
        self.z = 0.0
        self.bars = sorted([random.uniform(-4.0, 4.0) for _ in range(5)])
        self.get_logger().info(f"Bars at: {self.bars}")

        self.state = "forward"
        self.speed = 0.05

        self.jump_origin_x = 0.0
        self.jump_target_x = 0.0
        self.jump_peak_z = 0.6
        self.jump_progress = 0.0
        self.jump_duration = 20

        self.trail_points = []

    def timer_callback(self):
        self.update_state()
        self.publish_cube()
        
        self.publish_bars()
        self.publish_trail()

    def update_state(self):
        if self.x >= 10.0:
            self.get_logger().info("Reached grid boundary. Resetting cube.")
            self.x = -5.0
            self.z = 0.0
            self.state = "forward"
            self.bars = sorted([random.uniform(-4.0, 4.0) for _ in range(5)])
            self.trail_points.clear()
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

            self.x = (1 - self.jump_progress) * self.jump_origin_x + self.jump_progress * self.jump_target_x
            self.z = 4 * self.jump_peak_z * self.jump_progress * (1 - self.jump_progress)

        self.trail_points.append(Point(x=self.x, y=self.y, z=self.z))

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

    def publish_trail(self):
        trail = Marker()
        trail.header.frame_id = "map"
        trail.header.stamp = self.get_clock().now().to_msg()
        trail.ns = "trail"
        trail.id = 3
        trail.type = Marker.LINE_STRIP
        trail.action = Marker.ADD
        trail.scale.x = 0.03
        trail.color.r = 1.0
        trail.color.g = 1.0
        trail.color.b = 0.0
        trail.color.a = 0.8
        trail.points = self.trail_points[-100:]
        self.publisher.publish(trail)


def main(args=None):
    rclpy.init(args=args)
    node = MovingCubeWithTrail()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
