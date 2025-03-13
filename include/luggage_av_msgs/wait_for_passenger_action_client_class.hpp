#ifndef wait_for_passenger_action_CLIENT_HPP_
#define wait_for_passenger_action_CLIENT_HPP_

#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "luggage_av_msgs/action/WaitForPassenger.hpp"


class MinimalActionClient : public rclcpp::Node
{
public:
  // WaitForPassenger class
  using WaitForPassenger = luggage_av_msgs::action::WaitForPassenger;
  // GoalHandle class (handles the accept, the cancel, and the execute functions)
  using GoalHandle = rclcpp_action::ClientGoalHandle<WaitForPassenger>;

  explicit MinimalActionClient(const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions());

  void send_goal(int objective);

private:
  rclcpp_action::Client<WaitForPassenger>::SharedPtr client_ptr_;

  // Response Callback
  void goal_response_callback(GoalHandle::SharedPtr goal_message);
  // Feedback Callback
  void feedback_callback(GoalHandle::SharedPtr, const std::shared_ptr<const WaitForPassenger::Feedback> feedback_message);
  // Result Callback
  void result_callback(const GoalHandle::WrappedResult &result_message);
};

#endif  // wait_for_passenger_action_CLIENT_HPP_
