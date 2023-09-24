#include <functional>
#include <memory>
#include <thread>

#include "turtle_action/action/message_turtle_commands.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

#include "action_turtle_commands/visibility_control.h"

namespace turtle_action_cpp
{
class ActionTurtleServer : public rclcpp::Node
{
public:
  using TurtleCommands = turtle_action::action::MessageTurtleCommands;
  using GoalHandleTurtle = rclcpp_action::ServerGoalHandle<TurtleCommands>;

  ACTION_TURTLE_COMMANDS_CPP_PUBLIC
  explicit ActionTurtleServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("action_turtle_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<TurtleCommands>(
      this,
      "execute_turtle_commands",
      std::bind(&ActionTurtleServer::handle_goal, this, _1, _2),
      std::bind(&ActionTurtleServer::handle_cancel, this, _1),
      std::bind(&ActionTurtleServer::handle_accepted, this, _1));

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    subscription_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, std::bind(&ActionTurtleServer::pose_callback, this, _1));
  }

private:
  typedef struct {
    const std::shared_ptr<GoalHandleTurtle> goal_handle;
    geometry_msgs::msg::Twist request;
    const std::shared_ptr<TurtleCommands::Feedback> feedback;
    const std::shared_ptr<TurtleCommands::Result> result;
  } Context;

  rclcpp_action::Server<TurtleCommands>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;

  turtlesim::msg::Pose initial_pose_;
  turtlesim::msg::Pose current_pose_;

  void pose_callback(const turtlesim::msg::Pose & msg) {
    current_pose_ = msg;
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const TurtleCommands::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with distanse %f", goal->s);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleTurtle> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleTurtle> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&ActionTurtleServer::execute, this, _1), goal_handle}.detach();
  }


  void turn(Context& ctx, float theta, rclcpp::Rate& loop_rate) {
    initial_pose_ = current_pose_; 

    ctx.request.linear.x = 0;
    ctx.request.angular.z = theta/180*M_PI;
    publisher_->publish(ctx.request);
    
    auto end_theta = initial_pose_.theta + theta/180.0f*M_PI;
    RCLCPP_INFO(this->get_logger(), "Turn");
    while(abs(end_theta - current_pose_.theta) > 0.02 && rclcpp::ok()) {
      if (ctx.goal_handle->is_canceling()) {
        ctx.result->result = false;
        publisher_->publish(geometry_msgs::msg::Twist()); //stop turtle
        ctx.goal_handle->canceled(ctx.result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Publish feedback");
      RCLCPP_INFO(this->get_logger(), "End theta: %f, theta: %f", end_theta, current_pose_.theta);
      loop_rate.sleep();
    }
  }

  void forward(Context& ctx, float s, rclcpp::Rate& loop_rate) {
    float odom = 0;
    initial_pose_ = current_pose_; 
    auto request = geometry_msgs::msg::Twist();
    request.linear.x = s;
    publisher_->publish(request);

    while(abs(s - odom) > 0.02 && rclcpp::ok()) {
      if (ctx.goal_handle->is_canceling()) {
        ctx.result->result = false;
        publisher_->publish(geometry_msgs::msg::Twist()); //stop turtle
        ctx.goal_handle->canceled(ctx.result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      odom = pow(pow(current_pose_.x - initial_pose_.x, 2)+ pow(current_pose_.y - initial_pose_.y, 2), 0.5);
      ctx.feedback->odom = odom;
      ctx.goal_handle->publish_feedback(ctx.feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }
  }

  void execute(const std::shared_ptr<GoalHandleTurtle> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(10);
    const auto goal = goal_handle->get_goal();

    Context ctx = {
      goal_handle,
      geometry_msgs::msg::Twist(),
      std::make_shared<TurtleCommands::Feedback>(),
      std::make_shared<TurtleCommands::Result>()
    };

       

    if(goal->command == "turn_left"){
      turn(ctx, 90, loop_rate);
    }
    else if(goal->command == "turn_right") {
      turn(ctx, -90, loop_rate);
    }
    else if(goal->command != "forward") {
      RCLCPP_ERROR(this->get_logger(), "Unknown command: %s", goal->command.c_str());
      ctx.result->result = false;
      goal_handle->abort(ctx.result);
      RCLCPP_INFO(this->get_logger(), "Goal aborted");
      return;
    }

    forward(ctx, goal->s, loop_rate);

    turn(ctx, goal->angle, loop_rate);
    // Check if goal is done
    if (rclcpp::ok()) {
      ctx.result->result = true;
      goal_handle->succeed(ctx.result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};

}

RCLCPP_COMPONENTS_REGISTER_NODE(turtle_action_cpp::ActionTurtleServer)