#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/empty.hpp>
#include "luggage_av_msgs/action/send_route.hpp" 

using namespace std::chrono_literals;


const geometry_msgs::msg::PoseStamped DEFAULT_PICKUP_POSE = [] {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "luggage_av/map";
    pose.pose.position.x = 2.0;
    pose.pose.position.y = 1.0;
    return pose;
}();

const geometry_msgs::msg::PoseStamped DEFAULT_DROPOFF_POSE = [] {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "luggage_av/map";
    pose.pose.position.x = -2.0;
    pose.pose.position.y = -2.0;
    return pose;
}();

const geometry_msgs::msg::PoseStamped DEFAULT_OTHER_POSE = [] {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "luggage_av/map";
    pose.pose.position.x = -1.0;
    pose.pose.position.y = 2.0;
    return pose;
}();

const geometry_msgs::msg::PoseStamped DEFAULT_POSES[] = {
    DEFAULT_PICKUP_POSE,
    DEFAULT_DROPOFF_POSE,
    DEFAULT_OTHER_POSE,
};

class SendRouteClient : public rclcpp::Node
{
public:
  using SendRoute = luggage_av_msgs::action::SendRoute;
  using GoalHandleSendRoute = rclcpp_action::ClientGoalHandle<SendRoute>;

  SendRouteClient() : 
    rclcpp::Node("send_route_client", rclcpp::NodeOptions().arguments({"--ros-args", "-r", "__ns:=/luggage_av"}))

  {
    // Create the action client, which will look for "/luggage_av/send_route"
    client_ = rclcpp_action::create_client<SendRoute>(this, "send_route");
  }

  void send_goal()
  {
    RCLCPP_INFO(this->get_logger(), "Send Goal!");
    // Wait for the action server to be available
    if (!client_->wait_for_action_server(3s)) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      return;
    }

    // Create a goal
    auto goal = SendRoute::Goal();
    // Get pickup and dropoff poses from console input
    double pickup_x, pickup_y, dropoff_x, dropoff_y;
    std::cout << "Enter pickup pose: x y: ";
    std::cin >> pickup_x >> pickup_y;
    std::cout << "Enter dropoff pose: x y: ";
    std::cin >> dropoff_x >> dropoff_y;

    goal.pickup_pose.header.frame_id = "luggage_av/map";
    goal.pickup_pose.pose.position.x = pickup_x;
    goal.pickup_pose.pose.position.y = pickup_y;

    goal.dropoff_pose.header.frame_id = "luggage_av/map";
    goal.dropoff_pose.pose.position.x = dropoff_x;
    goal.dropoff_pose.pose.position.y = dropoff_y;
    // goal.pickup_pose = DEFAULT_PICKUP_POSE;
    // goal.dropoff_pose = DEFAULT_DROPOFF_POSE;

    // Send the goal
    auto send_goal_options = rclcpp_action::Client<SendRoute>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      [this](const GoalHandleSendRoute::SharedPtr & goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
          RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
      };

    send_goal_options.result_callback =
      [this](const GoalHandleSendRoute::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
        } else {
          RCLCPP_ERROR(this->get_logger(), "Goal failed");
        }
        rclcpp::shutdown();
      };

    client_->async_send_goal(goal, send_goal_options);
  }

private:
  rclcpp_action::Client<SendRoute>::SharedPtr client_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SendRouteClient>();
  node->send_goal();
  rclcpp::spin(node);
  // rclcpp::shutdown();
  return 0;
}