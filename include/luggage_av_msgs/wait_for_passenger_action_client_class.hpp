#ifndef wait_for_passenger_action_CLIENT_HPP_
#define wait_for_passenger_action_CLIENT_HPP_

#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "luggage_av_actions/action/luggage_av_actions.hpp"


class MinimalActionClient : public rclcpp::Node
{
public:
  // WaitForPassengerAction class
  using WaitForPassengerAction = luggage_av_actions::action::WaitForPassengerAction;
  // GoalHandle class (handles the accept, the cancel, and the execute functions)
  using GoalHandle = rclcpp_action::ClientGoalHandle<WaitForPassengerAction>;

  explicit MinimalActionClient(const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions());

  void send_goal(int objective);

private:
  rclcpp_action::Client<WaitForPassengerAction>::SharedPtr client_ptr_;

  // Response Callback
  void goal_response_callback(GoalHandle::SharedPtr goal_message);
  // Feedback Callback
  void feedback_callback(GoalHandle::SharedPtr, const std::shared_ptr<const WaitForPassengerAction::Feedback> feedback_message);
  // Result Callback
  void result_callback(const GoalHandle::WrappedResult &result_message);
};

#endif  // wait_for_passenger_action_CLIENT_HPP_
