#include <functional>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include "turtle_action/action/message_turtle_commands.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using goals_vector = std::shared_ptr<std::vector<turtle_action::action::MessageTurtleCommands::Goal>>;

class ActionTurtleClient : public rclcpp::Node
{
public:
  using TurtleCommands = turtle_action::action::MessageTurtleCommands;
  using GoalHandleTurtle = rclcpp_action::ClientGoalHandle<TurtleCommands>;

  explicit ActionTurtleClient(goals_vector goals, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("action_turtle_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<TurtleCommands>(
      this,
      "execute_turtle_commands");

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }
    goals_ = goals;
    send_goal(goals->at(0));
  }

  void send_goal(TurtleCommands::Goal goal_msg)
  {
    using namespace std::placeholders;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<TurtleCommands>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&ActionTurtleClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&ActionTurtleClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&ActionTurtleClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<TurtleCommands>::SharedPtr client_ptr_;
  goals_vector goals_;
  size_t finished = 0;

  void goal_response_callback(const GoalHandleTurtle::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleTurtle::SharedPtr,
    const std::shared_ptr<const TurtleCommands::Feedback> feedback)
  {
    
    RCLCPP_INFO(this->get_logger(), "Distance passed: %f", feedback->odom);
  }

  void result_callback(const GoalHandleTurtle::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
  
    RCLCPP_INFO(this->get_logger(), "Result received: %s", result.result ? "true" : "false");
    finished++;

    if(finished != goals_->size()){
      send_goal(goals_->at(finished));
    }
    else
      rclcpp::shutdown();
  }
};  // class ActionTurtleClient

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto goals = std::make_shared<std::vector<turtle_action::action::MessageTurtleCommands::Goal>>();

  auto goal_1 = turtle_action::action::MessageTurtleCommands::Goal();
  goal_1.command = "forward";
  goal_1.s = 2;
  goal_1.angle = 0;
  goals->push_back(goal_1);

  auto goal_2 = turtle_action::action::MessageTurtleCommands::Goal();
  goal_2.command = "turn_right";
  goal_2.s = 1;
  goal_2.angle = 45;
  goals->push_back(goal_2);
  
  auto client = std::make_shared<ActionTurtleClient>(goals);
  
  rclcpp::spin(client);

  rclcpp::shutdown();
  return 0;  
}