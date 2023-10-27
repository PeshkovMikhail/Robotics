#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>


#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"


using namespace std::chrono_literals;
using namespace geometry_msgs;
using std::placeholders::_1;

class DepthFollower : public rclcpp::Node
{
public:
	DepthFollower()
	: Node("depth_follower")
	{
        distance_ = this->declare_parameter<float>("distance", 1.0f);
		publisher_ = this->create_publisher<msg::Twist>("/robot_lidar/cmd_vel", 10);
		subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/robot_lidar/depth_image", 10,
             std::bind(&DepthFollower::topic_callback, this, _1));
	}
private:
	void topic_callback(const sensor_msgs::msg::Image & msg) {
        if(image == nullptr) {
            image = (float*) new float[msg.height*msg.width]();
        }
		memcpy(image, msg.data.data(), msg.step*msg.height);
		float min_distance = image[msg.width*msg.height/2 + msg.width/2];

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
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    float distance_;
    float* image = nullptr;
};


int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DepthFollower>());
	rclcpp::shutdown();
	return 0;
}