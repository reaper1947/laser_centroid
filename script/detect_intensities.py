import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import tf2_ros
import geometry_msgs.msg
from visualization_msgs.msg import Marker, MarkerArray

class DistanceCalculator(Node):
    def __init__(self):
        super().__init__('distance_calculator')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
        
        self.marker_publisher = self.create_publisher(MarkerArray, '/markers', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.last_object_time = None
        self.transforms = []  # You don't seem to need this anymore
        self.timer = self.create_timer(0.1, self.remove_transform)

    def lidar_callback(self, msg):
        intensity_threshold = 37
        proximity_threshold = 0.15

        detected_points = []

        # Filter points by intensity and calculate position for each point
        for i in range(len(msg.ranges)):
            if msg.intensities[i] > intensity_threshold and msg.ranges[i] < float('inf'):
                angle = msg.angle_min + i * msg.angle_increment
                x = msg.ranges[i] * math.cos(angle)
                y = msg.ranges[i] * math.sin(angle)
                detected_points.append((x, y))

        # Group points based on proximity
        grouped_points = []
        while detected_points:
            point = detected_points.pop(0)
            group = [point]
            for other_point in detected_points[:]:
                if self.calculate_distance(point, other_point) < proximity_threshold:
                    group.append(other_point)
                    detected_points.remove(other_point)
            grouped_points.append(group)

        # Create MarkerArray for Group 1 and Group 2
        marker_array = MarkerArray()

        # Process groups and generate markers
        if len(grouped_points) > 0:
            group_1 = grouped_points[0]
            x_avg1 = sum(p[0] for p in group_1) / len(group_1)
            y_avg1 = sum(p[1] for p in group_1) / len(group_1)
            self.create_marker(x_avg1, y_avg1, "group_1", marker_array)

        if len(grouped_points) > 1:
            group_2 = grouped_points[1]
            x_avg2 = sum(p[0] for p in group_2) / len(group_2)
            y_avg2 = sum(p[1] for p in group_2) / len(group_2)
            self.create_marker(x_avg2, y_avg2, "group_2", marker_array)

            # Calculate midpoint and send it as tf
            x_midpoint = (x_avg1 + x_avg2) / 2
            y_midpoint = (y_avg1 + y_avg2) / 2
            self.send_transform(x_midpoint, y_midpoint, "midpoint")

            # Calculate distance between Group 1 and Group 2
            distance = self.calculate_distance((x_avg1, y_avg1), (x_avg2, y_avg2))
            self.get_logger().info(f"Distance between Group 1 and Group 2: {distance:.2f} meters")

        # Publish MarkerArray
        self.marker_publisher.publish(marker_array)

        self.last_object_time = self.get_clock().now()

    def create_marker(self, x, y, frame_id, marker_array):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'laser'  # This ensures the frame of reference is 'laser'
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0  # Ensure this is float
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0 if frame_id == "group_1" else 0.0
        marker.color.g = 0.0 if frame_id == "group_1" else 1.0
        marker.color.b = 0.0

        marker.id = len(marker_array.markers)
        marker.ns = frame_id

        marker_array.markers.append(marker)

    def send_transform(self, x, y, child_frame_id):
        transform = geometry_msgs.msg.TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'laser'
        transform.child_frame_id = child_frame_id
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)

    def calculate_distance(self, point1, point2):
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

    def remove_transform(self):
        if self.last_object_time and (self.get_clock().now() - self.last_object_time).nanoseconds > 0.5 * 1e9:
            self.get_logger().warn("Removing transform for object.")
            self.last_object_time = None

def main(args=None):
    rclpy.init(args=args)
    distance_calculator = DistanceCalculator()
    rclpy.spin(distance_calculator)
    distance_calculator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
