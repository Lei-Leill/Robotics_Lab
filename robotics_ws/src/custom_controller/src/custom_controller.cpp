#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "franka_msgs/action/grasp.hpp"
#include "franka_msgs/action/move.hpp"

class CustomController : public rclcpp::Node
{
public:
  using Grasp = franka_msgs::action::Grasp;
  using Move = franka_msgs::action::Move;
  using GraspGoalHandle = rclcpp_action::ClientGoalHandle<Grasp>;
  using MoveGoalHandle = rclcpp_action::ClientGoalHandle<Move>;

  CustomController() : Node("custom_controller")
  {
    move_client_ = rclcpp_action::create_client<Move>(this, "/franka_gripper/move");
    grasp_client_ = rclcpp_action::create_client<Grasp>(this, "/franka_gripper/grasp");

    timer_ = this->create_wall_timer(std::chrono::seconds(2), std::bind(&CustomController::start_sequence, this));
  }

private:
  rclcpp_action::Client<Move>::SharedPtr move_client_;
  rclcpp_action::Client<Grasp>::SharedPtr grasp_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool sequence_started_ = false;

  void start_sequence()
  {
    if (sequence_started_) return;
    sequence_started_ = true;

    if (!move_client_->wait_for_action_server(std::chrono::seconds(5)) ||
        !grasp_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
      RCLCPP_ERROR(this->get_logger(), "Gripper action server not available.");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Sending open (move) goal...");

    auto move_goal = Move::Goal();
    move_goal.width = 0.08;
    move_goal.speed = 0.05;

    auto move_goal_options = rclcpp_action::Client<Move>::SendGoalOptions();
    move_goal_options.result_callback = [this](const MoveGoalHandle::WrappedResult & result) {
      RCLCPP_INFO(this->get_logger(), "Open complete, now closing (grasp)...");

      auto grasp_goal = Grasp::Goal();
      grasp_goal.width = 0.0;
      grasp_goal.speed = 0.05;
      grasp_goal.force = 20.0;
grasp_goal.epsilon.inner = 0.01;
grasp_goal.epsilon.outer = 0.01;

      auto grasp_goal_options = rclcpp_action::Client<Grasp>::SendGoalOptions();
      grasp_goal_options.result_callback = [this](const GraspGoalHandle::WrappedResult & result) {
        RCLCPP_INFO(this->get_logger(), "Gripper close complete.");
      };

      grasp_client_->async_send_goal(grasp_goal, grasp_goal_options);
    };

    move_client_->async_send_goal(move_goal, move_goal_options);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CustomController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

