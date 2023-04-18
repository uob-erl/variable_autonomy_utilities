#include <cstdio>
#include <chrono>
#include <memory>
#include <string>
#include <iostream>

// ROS specific libraries
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "actionlib_msgs/msg/goal_id.hpp"

class LoaSwitcher : public rclcpp::Node
{
public:
  LoaSwitcher(); // Constructor

private:
  // Declaration of functions

  // take the LoA decided by the agents or arbitration.
  // void loaCodeCallback(const std_msgs::msg::Int16 &loa_code_msg);      // in code format (e.g. a number)
  void loaStringCallback(const std_msgs::msg::String &loa_string_msg); // in string format (e.g. "teleoperation", "autonomy")

  // take the cmd velocity/control command from the different LoAs
  void teleopCallback(const geometry_msgs::msg::Twist &cmd_vel_teleop_msg) const;    // from joystick/teleoperation (ie. LoA = teleoperation)
  void sharedCallback(const geometry_msgs::msg::Twist &cmd_vel_shared_msg) const;    // from the shared control arbitrator node (ie. LoA = shared control)
  void autoNavCallback(const geometry_msgs::msg::Twist &cmd_vel_autonomy_msg) const; // from autonomous navigation (ie. LoA = autonomy)

  // Declare variables

  std::string current_loa_string_;
  int current_loa_code_;
  bool valid_loa_;

  actionlib_msgs::msg::GoalID cancelGoal_;
  geometry_msgs::msg::Twist cmd_vel_for_robot_;

  // Declare the subscriptions and publishers
  // rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr loa_code_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr loa_string_sub_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_teleop_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_shared_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_autonomy_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_for_robot_pub_; // This is the cmd_vel send to robot (e.g. motor drivers) to execute
  rclcpp::Publisher<actionlib_msgs::msg::GoalID>::SharedPtr cancelGoal_pub_;      // publishes an empty goal to cancel existing Nav goal (if any)
};

// Constructor

LoaSwitcher::LoaSwitcher() : Node("loa_switcher_node")

{

  // declare parameters and initialize variable defaults

  valid_loa_ = true;
  current_loa_string_ = "Stop"; //  stop/idle mode.

  cmd_vel_for_robot_.linear.x = 0;
  cmd_vel_for_robot_.angular.z = 0;

  // Setup/create Subscriptions and Publishers

  // loa_code_sub_ = this->create_subscription<std_msgs::msg::Int16>("loa_code", 5, std::bind(&LoaSwitcher::loaCodeCallback, this, std::placeholders::_1));
  loa_string_sub_ = this->create_subscription<std_msgs::msg::String>("loa_string", 5, std::bind(&LoaSwitcher::loaStringCallback, this, std::placeholders::_1));
  cmd_vel_teleop_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel_teleop", 5, std::bind(&LoaSwitcher::teleopCallback, this, std::placeholders::_1));
  cmd_vel_shared_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel_shared_control", 5, std::bind(&LoaSwitcher::sharedCallback, this, std::placeholders::_1));
  cmd_vel_autonomy_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel_autonomy", 5, std::bind(&LoaSwitcher::autoNavCallback, this, std::placeholders::_1));

  cmd_vel_for_robot_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  cancelGoal_pub_ = this->create_publisher<actionlib_msgs::msg::GoalID>("/move_base/cancel", 1);
}

// Functions and CallBacks

// reads the LoA topic (i.e. the LoA decided by the agents or arbitration) to inform class internal variable
// The bellow is if the string format for representing LoAs is used.
// You can add a new callback but for the code format if a code is used to represent each LoA.
// the logic should be identical
void LoaSwitcher::loaStringCallback(const std_msgs::msg::String &loa_string_msg)
{

  if (loa_string_msg.data == "Stop" && current_loa_string_ != "Stop")
  {
    current_loa_string_ = "Stop";
    valid_loa_ = true;
    cmd_vel_for_robot_.linear.x = 0;
    cmd_vel_for_robot_.angular.z = 0;
    cmd_vel_for_robot_pub_->publish(cmd_vel_for_robot_);
    cancelGoal_pub_->publish(cancelGoal_); // publishes an empty goal to cancel existing Nav goal (if any)
    RCLCPP_INFO(get_logger(), "Stop robot");
  }

  else if (loa_string_msg.data == "Teleoperation" && current_loa_string_ != "Teleoperation")
  {
    current_loa_string_ = "Teleoperation";
    valid_loa_ = true;
    cmd_vel_for_robot_.linear.x = 0;
    cmd_vel_for_robot_.angular.z = 0;
    cmd_vel_for_robot_pub_->publish(cmd_vel_for_robot_); // solves bug in which last cmd_vel msg from current LoA is propagated in the new LoA if switch happens
    RCLCPP_INFO(get_logger(), "LoA: Teleoperation");
  }

  else if (loa_string_msg.data == "Autonomy" && current_loa_string_ != "Autonomy")
  {
    current_loa_string_ = "Autonomy";
    valid_loa_ = true;
    cmd_vel_for_robot_.linear.x = 0;
    cmd_vel_for_robot_.angular.z = 0;
    cmd_vel_for_robot_pub_->publish(cmd_vel_for_robot_); // solves bug in which last cmd_vel msg from current LoA is propagated in the new LoA if switch happens
    RCLCPP_INFO(get_logger(), "LoA: Autonomy");
  }

  else if (loa_string_msg.data == "Shared_Control" && current_loa_string_ != "Shared_Control")
  {
    current_loa_string_ = "Shared_Control";
    valid_loa_ = true;
    cmd_vel_for_robot_.linear.x = 0;
    cmd_vel_for_robot_.angular.z = 0;
    cmd_vel_for_robot_pub_->publish(cmd_vel_for_robot_); // solves bug in which last cmd_vel msg from current LoA is propagated in the new LoA if switch happens
    RCLCPP_INFO(get_logger(), "LoA: Shared Control");
  }

  else
  {
    current_loa_string_ = loa_string_msg.data;
    valid_loa_ = false;
    RCLCPP_ERROR(get_logger(), current_loa_string_.c_str(), "%s is not a valid LoA. Check for spelling or other errors!");
  }
}

// Republishes the autonomy cmd_vel to the robot cmd_vel topic if autonomy is choosen
void LoaSwitcher::autoNavCallback(const geometry_msgs::msg::Twist &cmd_vel_autonomy_msg) const
{
  if (current_loa_string_ == "Autonomy")
  {
    cmd_vel_for_robot_pub_->publish(cmd_vel_autonomy_msg);
  }
}

// Republishes the teloperation cmd_vel to the robot cmd_vel topic if teleop is choosen
void LoaSwitcher::teleopCallback(const geometry_msgs::msg::Twist &cmd_vel_teleop_msg) const
{

  if (current_loa_string_ == "Teleoperation")
  {
    cmd_vel_for_robot_pub_->publish(cmd_vel_teleop_msg);
  }
}

// Republishes the Shared Control cmd_vel to the robot cmd_vel topic if teleop is choosen
void LoaSwitcher::sharedCallback(const geometry_msgs::msg::Twist &cmd_vel_shared_msg) const
{

  if (current_loa_string_ == "Shared_Control")
  {
    cmd_vel_for_robot_pub_->publish(cmd_vel_shared_msg);
  }
}

// Main function stuff
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LoaSwitcher>());
  rclcpp::shutdown();
  return 0;
}
