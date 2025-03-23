#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>
#include "luggage_av_msgs/action/send_route.hpp"

using namespace std::chrono_literals;
using SendRoute = luggage_av_msgs::action::SendRoute;
using GoalHandleSendRoute = rclcpp_action::ServerGoalHandle<SendRoute>;

class WaitForRoute : public rclcpp::Node
{
public:
  WaitForRoute()
  : rclcpp::Node("send_route_server", 
      rclcpp::NodeOptions().arguments({"--ros-args", "-r", "__ns:=/luggage_av"}))
  {
    // Create the action server on the "send_route" action.
    action_server_ = rclcpp_action::create_server<SendRoute>(
      this,
      "send_route",
      std::bind(&WaitForRoute::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&WaitForRoute::handle_cancel, this, std::placeholders::_1),
      std::bind(&WaitForRoute::handle_accepted, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "SendRoute action server started");
  }

private:
  // Called when a new goal request is received.
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const SendRoute::Goal> goal)
  {
    (void)uuid;
    RCLCPP_INFO(this->get_logger(),
      "Received goal: pickup x: %.2f, dropoff x: %.2f",
      goal->pickup_pose.pose.position.x,
      goal->dropoff_pose.pose.position.x);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // Called when a cancel request is received.
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleSendRoute> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // Called once the goal has been accepted.
  void handle_accepted(const std::shared_ptr<GoalHandleSendRoute> goal_handle)
  {
    // Execute the goal asynchronously.
    std::thread{[this, goal_handle]() {
      RCLCPP_INFO(this->get_logger(), "Executing goal...");
      // Simulate some processing delay.
      std::this_thread::sleep_for(3s);
      // Create a result message.
      auto result = std::make_shared<SendRoute::Result>();
      result->success = false;
      // (Optional) populate the result fields if needed.
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal execution completed");
    }}.detach();
  }

  rclcpp_action::Server<SendRoute>::SharedPtr action_server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto server_node = std::make_shared<WaitForRoute>();
  rclcpp::spin(server_node);
  rclcpp::shutdown();
  return 0;
}
