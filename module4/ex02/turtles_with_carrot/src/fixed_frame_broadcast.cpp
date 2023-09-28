#include <chrono>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class FixedFrameBroadcaster : public rclcpp::Node
{
public:
  FixedFrameBroadcaster()
  : Node("fixed_frame_tf2_broadcaster")
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    int d = this->declare_parameter<int>("direction_of_rotation", 1);
    radius_ = this->declare_parameter<float>("radius", 2.0f);
    if(d != 1 && d !=-1){
      RCLCPP_ERROR(get_logger(), "Directino of rotation must be -1 or 1");
      rclcpp::shutdown();
    }
    direction_of_rotation_ = d;
    RCLCPP_INFO(this->get_logger(), "direction: %f\r\nradius: %f", direction_of_rotation_, radius_);
    timer_ = this->create_wall_timer(
      100ms, std::bind(&FixedFrameBroadcaster::broadcast_timer_callback, this));
  }

private:
  void broadcast_timer_callback()
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    auto time = t.header.stamp.sec;
    t.header.frame_id = "turtle1";
    t.child_frame_id = "carrot1";
    t.transform.translation.x = radius_*cos(direction_of_rotation_*time);
    t.transform.translation.y = radius_*sin(direction_of_rotation_*time);
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(t);
  }

rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  int direction_of_rotation_;
  float radius_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FixedFrameBroadcaster>());
  rclcpp::shutdown();
  return 0;
}