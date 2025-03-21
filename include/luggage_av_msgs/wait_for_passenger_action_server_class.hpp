#ifndef wait_for_passenger_action_SERVER_HPP_
#define wait_for_passenger_action_SERVER_HPP_

#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "luggage_av_actions/action/luggage_av_actions.hpp"

class MinimalActionServer : public rclcpp::Node
{
public:
  // WaitForPassengerAction class
  using WaitForPassengerAction = luggage_av_actions::action::WaitForPassengerAction;
  // GoalHandle class (handles the accept, the cancel, and the execute functions)
  using GoalHandle = rclcpp_action::ServerGoalHandle<WaitForPassengerAction>;

  explicit MinimalActionServer(const rclcpp::NodeOptions &action_server_options = rclcpp::NodeOptions());

private:
  // Action Server
  rclcpp_action::Server<WaitForPassengerAction>::SharedPtr action_server_;

  // Handle Goal Request
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const WaitForPassengerAction::Goal> goal_request);

  // Handle cancel
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle_canceled_);

  // Handle Accepted
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle_accepted_);

  // Execute
  void execute(const std::shared_ptr<GoalHandle> goal_handle_);
};

#endif  // wait_for_passenger_action_SERVER_HPP_
