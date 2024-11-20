#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <vector>
#include <cmath>
#include <algorithm>

class Controller : public rclcpp::Node {
private:
    // ROS 2 Subscribers and Publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_pub_;

    // Variables
    ackermann_msgs::msg::AckermannDriveStamped ackermann_;
    double threshold_ = 1.0;
    double car_width_ = 0.8;
    double k_speed_ = 3.5;
    double k_speed2_ = 1.7;
    double k_angular_speed_ = 0.02;
    double min_ttc_ = 0.15;
    float speed_x_ = 0.0;
    int collision_avoid_angle_ = 0;

    void getSpeedCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
        speed_x_ = odom_msg->twist.twist.linear.x;
    }

    void ttcCalculation(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        float distance, velocity, ttc;
        float current_angle = 0.0;

        for (int i = 270; i < 405; ++i) {
            distance = scan_msg->ranges[i];
            velocity = speed_x_ * std::cos(current_angle);
            ttc = std::abs(distance / (velocity + 0.00001));
            current_angle = i * 0.005823;

            if (ttc < min_ttc_) {
                collision_avoid_angle_ = 30;
                break;
            }
        }

        for (int i = 675; i < 810; ++i) {
            distance = scan_msg->ranges[i];
            velocity = speed_x_ * std::cos(current_angle);
            ttc = std::abs(distance / (velocity + 0.00001));
            current_angle = i * 0.005823;

            if (ttc < min_ttc_) {
                collision_avoid_angle_ = -30;
                break;
            }
        }
    }

    void findCliff(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        ttcCalculation(scan_msg);

        int furthest_point = 0;
        double furthest_distance = 0.0;
        std::vector<int> cliffs;
        std::vector<int> obstacle_directions;
        std::vector<double> cliff_distances;
        std::vector<double> ranges(1080);

        for (int i = 0; i < 1080; ++i) {
            ranges[i] = scan_msg->ranges[i];
        }

        for (int i = 135; i < 945; ++i) {
            double current_distance = ranges[i];

            if (std::abs(current_distance - ranges[i - 1]) > threshold_) {
                if (current_distance > ranges[i - 1]) {
                    cliffs.push_back(i - 1);
                    cliff_distances.push_back(ranges[i - 1]);
                    obstacle_directions.push_back(2);  // Right
                } else {
                    cliffs.push_back(i);
                    cliff_distances.push_back(ranges[i]);
                    obstacle_directions.push_back(1);  // Left
                }
            }
        }

        for (size_t i = 0; i < cliffs.size(); ++i) {
            int n_readings = std::ceil(0.5 * car_width_ / (cliff_distances[i] * 0.005823));

            if (obstacle_directions[i] == 1) {
                for (int j = 0; j < n_readings; ++j) {
                    if (cliffs[i] + j < 1080) {
                        ranges[cliffs[i] + j] = 0.0;
                    }
                }
            } else if (obstacle_directions[i] == 2) {
                for (int j = 0; j < n_readings; ++j) {
                    if (cliffs[i] - j >= 0) {
                        ranges[cliffs[i] - j] = 0.0;
                    }
                }
            }
        }

        for (int i = 135; i < 945; ++i) {
            double current_distance = ranges[i];
            if (current_distance > furthest_distance) {
                furthest_point = i;
                furthest_distance = current_distance;
            }
        }

        pidController(furthest_point, furthest_distance, ranges);
    }

    void pidController(int furthest_point, double furthest_distance, std::vector<double>& ranges) {
        ackermann_.drive.steering_angle = (furthest_point - 540 + collision_avoid_angle_) * 0.005823;
        collision_avoid_angle_ = 0;

        if (std::abs(ackermann_.drive.steering_angle) < M_PI / 5) {
            ackermann_.drive.steering_angle /= 5;
        }

        if (furthest_distance > 30 && ackermann_.drive.steering_angle < M_PI / 10) {
            ackermann_.drive.speed = 10.0;
        } else {
            ackermann_.drive.speed = k_speed_ * std::abs(k_speed2_ - ackermann_.drive.steering_angle);
        }

        ackermann_.header.frame_id = "laser";
        ackermann_.header.stamp = this->now();
        ackermann_pub_->publish(ackermann_);
    }

public:
    Controller() : Node("controller") {
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&Controller::findCliff, this, std::placeholders::_1));
        ackermann_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&Controller::getSpeedCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Controller Node Initialized");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller>());
    rclcpp::shutdown();
    return 0;
}

