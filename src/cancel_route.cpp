#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;

class Nav2CancelClient : public rclcpp::Node
{
public:
  Nav2CancelClient() : Node("nav2_cancel_client")
  {
    client_ = rclcpp_action::create_client<NavigateToPose>(
      this,
      "/luggage_av/navigate_to_pose");

    // Single-shot cancellation
    cancel_goal();
  }

  void cancel_goal()
  {
    if (!client_->wait_for_action_server(3s)) {
      RCLCPP_ERROR(get_logger(), "Action server not available");
      rclcpp::shutdown();
      return;
    }

    auto future_cancel = client_->async_cancel_all_goals();
    
    if (rclcpp::spin_until_future_complete(
          this->get_node_base_interface(),
          future_cancel) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed to cancel goal");
    }
    else {
      RCLCPP_INFO(get_logger(), "Goal canceled successfully");
    }

    // Immediate shutdown after operation
    rclcpp::shutdown();
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Nav2CancelClient>();
  rclcpp::spin(node);
  return 0;
}