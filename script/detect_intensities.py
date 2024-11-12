import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import tf2_ros
import geometry_msgs.msg

class DistanceCalculator(Node):
    def __init__(self):
        super().__init__('distance_calculator')
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
        intensity_threshold = 37
        proximity_threshold = 0.15  

        self.transforms = []  # Clear old transforms
        detected_points = []

        # Filter points by intensity and calculate transform for each point
        for i in range(len(msg.ranges)):
            if msg.intensities[i] > intensity_threshold and msg.ranges[i] < float('inf'):
                angle = msg.angle_min + i * msg.angle_increment
                x = msg.ranges[i] * math.cos(angle)
                y = msg.ranges[i] * math.sin(angle)
                detected_points.append((x, y))
                self.transforms.append((x, y))  

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

        # Broadcast averaged positions for Group 1 and Group 2
        if len(grouped_points) > 0:
            group_1 = grouped_points[0]
            x_avg1 = sum(p[0] for p in group_1) / len(group_1)
            y_avg1 = sum(p[1] for p in group_1) / len(group_1)
            self.send_transform(x_avg1, y_avg1, "group_1")


        if len(grouped_points) > 1:
            group_2 = grouped_points[1]
            x_avg2 = sum(p[0] for p in group_2) / len(group_2)
            y_avg2 = sum(p[1] for p in group_2) / len(group_2)
            self.send_transform(x_avg2, y_avg2, "group_2")

            distance = self.calculate_distance((x_avg1, y_avg1), (x_avg2, y_avg2))
            r_mid = distance/2 
            if distance > 0:
                self.get_logger().info(f"L1: {distance:.2f} meters")
                self.get_logger().info(f"r mid {r_mid:.2f} meters")
                # Calculate the midpoint on the Y-axis and broadcast it as a transform
                y_midpoint = (y_avg1 + y_avg2) / 2
                x_midpoint = (x_avg1 + x_avg2) / 2
                self.send_transform(x_midpoint, y_midpoint, "midpoint")
                self.get_logger().info(f"midpoint cordinate: ({x_midpoint}, {y_midpoint})")

                # Calculate angles A, B, C
                a = math.sqrt(((x_avg1 - 0)**2) + ((y_avg1 - 0)**2))
                b = math.sqrt(((x_avg2 - 0)**2) + ((y_avg2 - 0)**2))
                c = distance

                print("ab ", a)
                print("ac ", b)
                print("bc ", c)       
                angle_A_deg, angle_B_deg, angle_C_deg = self.calculate_angles(a, b, c)
                print(f"angle A: {angle_A_deg:.2f}°")
                print(f"angle B: {angle_B_deg:.2f}°")
                print(f"angle C: {angle_C_deg:.2f}°")


        self.last_object_time = self.get_clock().now()

    def calculate_angles(self, a, b, c):
        # Calculate angles A, B, C in radians
        cos_value_A = (b**2 + c**2 - a**2) / (2 * b * c)
        cos_value_B = (a**2 + c**2 - b**2) / (2 * a * c)
        cos_value_C = (a**2 + b**2 - c**2) / (2 * a * b)

        # Safe acos
        angle_A_rad = self.safe_acos(cos_value_A)
        angle_B_rad = self.safe_acos(cos_value_B)
        angle_C_rad = self.safe_acos(cos_value_C)

        # Convert radians to degrees
        return math.degrees(angle_A_rad), math.degrees(angle_B_rad), math.degrees(angle_C_rad)

    def safe_acos(self, value):
        return math.acos(max(-1, min(1, value)))

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
        transform.transform.rotation.z = 1.0
        transform.transform.rotation.w = 0.0
        self.tf_broadcaster.sendTransform(transform)

    def send_transform2(self, x, y, child_frame_id):
        transform = geometry_msgs.msg.TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = child_frame_id
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 1.0
        transform.transform.rotation.w = 0.0
        self.tf_broadcaster.sendTransform(transform)
    def calculate_distance(self, point1, point2):
        return math.dist(point1, point2)

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
