import rclpy

from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Point

from visualization_msgs.msg import Marker

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data

import math



class PoseAndMarkerPublisher(Node):

    def __init__(self):

        super().__init__('pose_and_marker_publisher')



        # Pose publisher (can use sensor QoS)

        self.pose_pub = self.create_publisher(PoseStamped, 'pose', qos_profile_sensor_data)



        # Marker publisher with reliable QoS (for RViz2)

        marker_qos = QoSProfile(

            reliability=ReliabilityPolicy.RELIABLE,

            durability=DurabilityPolicy.VOLATILE,

            depth=10

        )

        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', marker_qos)



        # Timer to periodically publish markers and pose

        self.timer = self.create_timer(1.0, self.publish)



    def publish(self):

        now = self.get_clock().now().to_msg()



        # ---- PoseStamped message (fixed location) ----

        pose = PoseStamped()

        pose.header.stamp = now

        pose.header.frame_id = "map"

        pose.pose.position.x = 1.0

        pose.pose.position.y = 0.0

        pose.pose.position.z = 0.0

        pose.pose.orientation.w = 1.0

        self.pose_pub.publish(pose)



        # ---- Marker message (Speed Bumps on X-axis) ----

        marker = Marker()

        marker.header.stamp = now

        marker.header.frame_id = "map"

        marker.ns = "speed_bumps"

        marker.id = 0

        marker.type = Marker.LINE_LIST

        marker.action = Marker.ADD

        marker.scale.x = 0.05  # Line width

        marker.color.r = 1.0

        marker.color.g = 0.0

        marker.color.b = 0.0

        marker.color.a = 1.0

        marker.pose.orientation.w = 1.0

        marker.points = []



        # Create 4 speed bumps along the X-axis using a sine wave shape

        bump_spacing = 2.0  # Distance between the bumps on the X-axis

        bump_height = 1.0   # Height of the bump

        segments = 20       # Number of segments per bump



        for i in range(4):

            x_offset = i * bump_spacing  # Position of each bump on X-axis

            for j in range(segments - 1):

                # First point of the segment

                y1 = bump_height * math.sin(math.pi * (j / (segments - 1) - 0.5))

                y2 = bump_height * math.sin(math.pi * ((j + 1) / (segments - 1) - 0.5))

                x = x_offset

                z = 0.0



                marker.points.append(Point(x=x, y=y1, z=z))

                marker.points.append(Point(x=x, y=y2, z=z))



        # Publish the marker

        self.marker_pub.publish(marker)



def main(args=None):

    rclpy.init(args=args)

    node = PoseAndMarkerPublisher()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()



if __name__ == '__main__':

    main()

