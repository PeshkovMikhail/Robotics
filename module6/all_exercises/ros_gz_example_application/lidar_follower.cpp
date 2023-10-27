
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>


#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;
using namespace geometry_msgs;
using std::placeholders::_1;

class LidarFollower : public rclcpp::Node
{
public:
	LidarFollower()
	: Node("lidar_follower")
	{
        distance_ = this->declare_parameter<float>("distance", 1.0f);
		publisher_ = this->create_publisher<msg::Twist>("/robot_lidar/cmd_vel", 10);
		subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/robot_lidar/scan", 10,
             std::bind(&LidarFollower::topic_callback, this, _1));
	}
private:
	void topic_callback(const sensor_msgs::msg::LaserScan & msg) const {
        int rays_count = (msg.angle_max - msg.angle_min)/msg.angle_increment;
        float min_distance = msg.range_max;
        for(int i = 0; i < rays_count; i++) {
            if(msg.ranges[i] < msg.range_min && msg.ranges[i] > msg.range_max) {continue;}

            if(msg.ranges[i] < min_distance) {
                min_distance = msg.ranges[i];
            }
        }

        RCLCPP_INFO(this->get_logger(), "Current dist: %f", min_distance);
		
        auto message = msg::Twist();
        auto linear = msg::Vector3();
        if(min_distance > distance_){
            linear.x = -0.5;
        }
        else{
            linear.x = 0;
        }
		message.set__linear(linear);
		publisher_->publish(message);
	}

	rclcpp::Publisher<msg::Twist>::SharedPtr publisher_;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    float distance_;
};

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<LidarFollower>());
	rclcpp::shutdown();
	return 0;
}