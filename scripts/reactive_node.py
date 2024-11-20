#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class ReactiveFollowGap(Node):
    def __init__(self):
        super().__init__('reactive_follow_gap')
        
        # Topics & Subscriptions, Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # Subscribe to LiDAR
        self.lidar_sub = self.create_subscription(
            LaserScan, lidarscan_topic, self.lidar_callback, 10)
        
        # Publish to drive
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        # Initialize state variables
        self.angles = None
        self.angles_initialized = False
        self.get_logger().info("Reactive Follow Gap Node Initialized")

    def preprocess_lidar(self, data, window_size=7):
        """Preprocess the LiDAR scan array."""
        # Initialize data
        steering_viewport = 70  # in degrees
        ranges = np.array(data.ranges)
        if not self.angles_initialized:
            min_angle = data.angle_min
            max_angle = data.angle_max
            self.angles = np.linspace(min_angle, max_angle, ranges.shape[0])
            self.angles_initialized = True
            self.good_angle_idx = np.where(np.logical_and(
                self.angles > np.radians(-steering_viewport),
                self.angles < np.radians(steering_viewport)))
            self.angles = self.angles[self.good_angle_idx]

        # Handle NaNs and Infs
        ranges[np.isnan(ranges)] = 0
        good_idx = np.isfinite(ranges)
        ranges[~good_idx] = 5  # Replace Inf with 5 (not zero to retain gaps)

        # Smooth ranges
        kernel = np.ones((1, window_size)) / window_size
        smoothed_ranges = np.convolve(ranges, kernel[0], 'same')
        smoothed_ranges[~good_idx] = 0
        smoothed_ranges = np.clip(smoothed_ranges, 0, 3)
        smoothed_ranges = smoothed_ranges[self.good_angle_idx]
        return smoothed_ranges

    def find_max_gap(self, free_space_ranges):
        """Find the start and end indices of the max gap in free_space_ranges."""
        temp_arr = np.copy(free_space_ranges)
        temp_arr[np.nonzero(temp_arr)] = 2
        split = np.split(np.arange(free_space_ranges.shape[0]),
                         np.where(np.abs(np.diff(temp_arr)) >= 1)[0] + 1)

        sorted_split = sorted(split, key=len, reverse=True)
        for segment in sorted_split:
            if np.any(free_space_ranges[segment]):
                return np.min(segment), np.max(segment + 1)

    def find_best_point(self, start_i, end_i, ranges):
        """Find the best point (furthest) within the max gap."""
        return self.angles[np.argmax(ranges[start_i:end_i]) + start_i]

    def set_bubble(self, ranges, closest_point_idx, rb=0.6):
        """Set a safety bubble around the closest point."""
        angle = self.angles[closest_point_idx]
        dtheta = np.arctan2(rb, ranges[closest_point_idx])

        bubble_idx = np.where(
            np.logical_and(self.angles > angle - dtheta, self.angles < angle + dtheta))
        ranges[bubble_idx] = 0
        return ranges

    def lidar_callback(self, data):
        """Process each LiDAR scan using the Follow Gap algorithm."""
        proc_ranges = self.preprocess_lidar(data)

        # Find the closest point
        closest_point_idx = np.argmin(proc_ranges[np.nonzero(proc_ranges)])
        self.get_logger().info(f"Closest Point Index: {closest_point_idx}")

        # Eliminate all points inside the 'bubble'
        bubbled_ranges = self.set_bubble(proc_ranges, closest_point_idx)

        # Find the largest gap
        gap_start, gap_end = self.find_max_gap(bubbled_ranges)

        # Find the best point in the gap
        desired_angle = self.find_best_point(gap_start, gap_end, bubbled_ranges)

        # Calculate gap length and adjust speed
        max_gap_length = gap_end - gap_start  # Length of the largest gap
        max_speed = 4.0  # Maximum speed
        min_speed = 1.5  # Minimum speed
        speed = min_speed + (max_gap_length / len(bubbled_ranges)) * (max_speed - min_speed)

        # Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = desired_angle
        drive_msg.drive.speed = speed  # Dynamically calculated speed

        self.get_logger().info(f"Desired Angle: {desired_angle}, Speed: {speed}")
        self.drive_pub.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    reactive_follow_gap = ReactiveFollowGap()
    rclpy.spin(reactive_follow_gap)
    reactive_follow_gap.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

