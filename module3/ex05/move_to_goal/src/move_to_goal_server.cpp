#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "goal_action/action/goal_coords.hpp"

#include "move_to_goal/visibility_control.h"

namespace move_to_goal_action_cpp
{
class MoveToGoalActionServer : public rclcpp::Node
{
public:
  using GoalCoords = goal_action::action::GoalCoords;
  using GoalHandle = rclcpp_action::ServerGoalHandle<GoalCoords>;

  ACTION_MOVE_TO_GOAL_CPP_PUBLIC
  explicit MoveToGoalActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("move_to_goal_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<GoalCoords>(
      this,
      "execute_move_to_goal",
      std::bind(&MoveToGoalActionServer::handle_goal, this, _1, _2),
      std::bind(&MoveToGoalActionServer::handle_cancel, this, _1),
      std::bind(&MoveToGoalActionServer::handle_accepted, this, _1));

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    subscription_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, std::bind(&MoveToGoalActionServer::pose_callback, this, _1));
  }

private:
  typedef struct {
    float x;
    float y;
  } Coords;

  rclcpp_action::Server<GoalCoords>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;


  bool initial_pose_taken_ = false;


  float euclidian_distance(Coords goal_pose) {
        return pow(pow(goal_pose.x - current_pose.x, 2) + pow(goal_pose.y - current_pose.y, 2), 0.5);
    }

    float linear_vel(Coords goal_pose, float constant=1.5f) {
        return constant * euclidian_distance(goal_pose);
    }

    float steering_angle(Coords goal_pose) {
        return atan2(goal_pose.y - current_pose.y, goal_pose.x - current_pose.x);
    }

    float anglular_vel(Coords goal_pose, float constant = 6) {
        return constant * (steering_angle(goal_pose)- current_pose.theta);
    }

    void pose_callback(const turtlesim::msg::Pose & msg) {
        //RCLCPP_INFO(this->get_logger(), "pose taken");
        current_pose = msg;
        // current_pose.x = round(current_pose.x, 4);
    }

    void execute(const std::shared_ptr<GoalHandle> goal_handle) { 
        RCLCPP_INFO(this->get_logger(), "hello there");
        auto goal = goal_handle->get_goal();

        auto feedback = std::make_shared<GoalCoords::Feedback>();
        auto result = std::make_shared<GoalCoords::Result>();

        Coords coords = {
          goal->x, goal->y
        };

        rclcpp::Rate loop_rate(10);
        geometry_msgs::msg::Twist twist;
        while(euclidian_distance(coords) >= tolerance_ && rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
              result->result = false;
              publisher_->publish(geometry_msgs::msg::Twist()); //stop turtle
              goal_handle->canceled(result);
              RCLCPP_INFO(this->get_logger(), "Goal canceled");
              return;
            }

            feedback->dist = 0;
            goal_handle->publish_feedback(feedback);
            twist.linear.x = linear_vel(coords);
            twist.angular.z = anglular_vel(coords);
            publisher_->publish(twist);
            loop_rate.sleep();
        }
        RCLCPP_INFO(this->get_logger(), "done");
        twist.angular.z = 0;
        twist.linear.x = 0;
        publisher_->publish(twist);
        

        if (rclcpp::ok()) {
          result->result = true;
          goal_handle->succeed(result);
          RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
    }

    turtlesim::msg::Pose current_pose = turtlesim::msg::Pose();
    turtlesim::msg::Pose goal_;
    float tolerance_=0.3;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GoalCoords::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with x: %f, y: %f", goal->x, goal->y);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&MoveToGoalActionServer::execute, this, _1), goal_handle}.detach();
  }
  
};  // class FibonacciActionServer

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(move_to_goal_action_cpp::MoveToGoalActionServer)