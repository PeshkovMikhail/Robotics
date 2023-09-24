#include <functional>
#include <future>
#include <memory>
#include <string>

#include "goal_action/action/goal_coords.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"




class MoveToGoalActionClient : public rclcpp::Node
{
public:
  using GoalCoords = goal_action::action::GoalCoords;
  using GoalHandle = rclcpp_action::ClientGoalHandle<GoalCoords>;

  explicit MoveToGoalActionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("move_to_goal_action_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<GoalCoords>(
      this,
      "execute_move_to_goal");

    RCLCPP_INFO(this->get_logger(), "hello there");
  }

  auto send_goal(GoalCoords::Goal goal_msg)
  {
    using namespace std::placeholders;
    //this->timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "hello there");

    if (!this->client_ptr_->wait_for_action_server()) {
      std::cerr << "failed" << std::endl;
      RCLCPP_INFO(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<GoalCoords>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&MoveToGoalActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&MoveToGoalActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&MoveToGoalActionClient::result_callback, this, _1);
    return this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<GoalCoords>::SharedPtr client_ptr_;

  void goal_response_callback(const GoalHandle::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandle::SharedPtr,
    const std::shared_ptr<const GoalCoords::Feedback> feedback)
  {
    
    RCLCPP_INFO(this->get_logger(), "Distance passed: %f", feedback->dist);
  }

  void result_callback(const GoalHandle::WrappedResult & result)
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
    rclcpp::shutdown();
  }
};  // class GoalCoordsActionClient

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveToGoalActionClient>();
    if(argc != 4) {
        std::cerr << "Usage: move_to_goal x y theta" << std::endl;
        rclcpp::shutdown();
    }
    goal_action::action::GoalCoords::Goal g;
    g.x = atof(argv[1]);
    g.y = atof(argv[2]);
    g.theta = atof(argv[2]);
    
    auto future = node->send_goal(g);
    rclcpp::spin_until_future_complete(node, future);
    rclcpp::shutdown();
    return 0;
}