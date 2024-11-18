import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import tf2_ros
import geometry_msgs.msg
import tf_transformations


class DistanceCalculator(Node):
    def __init__(self):
        super().__init__('distance_calculator')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_filtered',
            self.lidar_callback,
            10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.last_object_time = None
        self.timer = self.create_timer(0.1, self.remove_transform)
        self.transforms = []  # Store transforms instead of broadcasting immediately

    def lidar_callback(self, msg):
        intensity_threshold = 235
        proximity_threshold = 0.13

        detected_points = self.filter_points(msg, intensity_threshold)
        grouped_points = self.group_points(detected_points, proximity_threshold)

        self.broadcast_group_transforms(grouped_points)

        if len(grouped_points) >= 4:
            x_avg1 = sum(p[0] for p in grouped_points[0]) / len(grouped_points[0])
            y_avg1 = sum(p[1] for p in grouped_points[0]) / len(grouped_points[0])

            x_avg2 = sum(p[0] for p in grouped_points[1]) / len(grouped_points[1])
            y_avg2 = sum(p[1] for p in grouped_points[1]) / len(grouped_points[1])

            x_avg3 = sum(p[0] for p in grouped_points[2]) / len(grouped_points[2])
            y_avg3 = sum(p[1] for p in grouped_points[2]) / len(grouped_points[2])

            x_avg4 = sum(p[0] for p in grouped_points[3]) / len(grouped_points[3])
            y_avg4 = sum(p[1] for p in grouped_points[3]) / len(grouped_points[3])

            x_centroid = (x_avg1 + x_avg2 + x_avg3 + x_avg4) / 4
            y_centroid = (y_avg1 + y_avg2 + y_avg3 + y_avg4) / 4
            angle = math.atan2(y_centroid, x_centroid)
            angle_degrees = math.degrees(angle)
            quaternion = tf_transformations.quaternion_from_euler(0, 0, angle)  # (roll, pitch, yaw)

            # x_centroid_adjusted = x_centroid * math.cos(angle) - y_centroid * math.sin(angle)
            # y_centroid_adjusted = x_centroid * math.sin(angle) + y_centroid * math.cos(angle)
            self.get_logger().info(f"angle {angle_degrees:.2f} c")
            self.send_transform3(x_centroid, y_centroid, "centroid",
                                 quaternion)
            self.get_logger().info(f"Centroid coordinate: ({x_centroid}, {y_centroid})")
            distance = self.calculate_distance((x_avg4, y_avg4), (x_avg1, y_avg1))
            distance2 = self.calculate_distance((x_avg4, y_avg4), (x_avg3, y_avg3))
            self.get_logger().info(f"outer_width: {distance:.2f}")
            self.get_logger().info(f"outer_length: {distance2:.2f}")
        self.last_object_time = self.get_clock().now()

    def filter_points(self, msg, intensity_threshold):
        detected_points = []
        for i in range(len(msg.ranges)):
            if msg.intensities[i] > intensity_threshold and msg.ranges[i] < float('inf'):
                angle = msg.angle_min + i * msg.angle_increment
                x = msg.ranges[i] * math.cos(angle)
                y = msg.ranges[i] * math.sin(angle)
                detected_points.append((x, y))
        return detected_points

    def group_points(self, detected_points, proximity_threshold):
        grouped_points = []
        while detected_points:
            point = detected_points.pop(0)
            group = [point]
            for other_point in detected_points[:]:
                if self.calculate_distance(point, other_point) < proximity_threshold:
                    group.append(other_point)
                    detected_points.remove(other_point)
            grouped_points.append(group)
        return grouped_points

    def broadcast_group_transforms(self, grouped_points):
        for i in range(min(4, len(grouped_points))):
            group = grouped_points[i]
            x_avg = sum(p[0] for p in group) / len(group)
            y_avg = sum(p[1] for p in group) / len(group)
            self.send_transform(x_avg, y_avg, f"group_{i}")

    def send_transform(self, x, y, child_frame_id):
        transform = geometry_msgs.msg.TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'laser'
        transform.child_frame_id = child_frame_id
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)

    def send_transform3(self, x, y, child_frame_id, quaternion):
        transform = geometry_msgs.msg.TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'laser'
        transform.child_frame_id = child_frame_id
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        self.tf_broadcaster.sendTransform(transform)

    def calculate_distance(self, point1, point2):
        return math.dist(point1, point2)

    def remove_transform(self):
        if self.last_object_time and (self.get_clock().now(
        ) - self.last_object_time).nanoseconds > 0.5 * 1e9:
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
