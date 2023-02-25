#include <cstdio>
#include <chrono>
#include <memory>
#include <string>

// ROS specific libraries
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"

// Create the node class MinimalPublisher by inheriting from rclcpp::Node.
// I think this is used in instead of ROS1 node handles
// Every this in the code is referring to the node.

class TeleopMobileRobot : public rclcpp::Node
{
public:
  TeleopMobileRobot(); // Constructor

private:
  // Declaration of functions
  void joyCallback(const sensor_msgs::msg::Joy &joy_msg) const;

  //  void publish();

  // Declare rclcpp object parameters variables
  rclcpp::Parameter param_axis_linear_;
  rclcpp::Parameter param_axis_angular_;
  rclcpp::Parameter param_scaling_linear_;
  rclcpp::Parameter param_scaling_angular_;
  rclcpp::Parameter param_teleop_button_;
  rclcpp::Parameter param_auto_button_;
  rclcpp::Parameter param_shared_control_button_;
  rclcpp::Parameter param_stop_button_;

  // int linear_axis_, angular_axis_, control_button_, stop_button_, shared_button_ ,auto_button_, teleop_button_;
  // double linear_scaling_, angular_scaling_;
  // geometry_msgs::Twist last_msg_published_;

  //  boost::mutex publish_mutex_;

  // rclcpp::Publisher vel_pub_, loa_pub_;

  // Declare the subscriptions and publishers
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr loa_code_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr loa_name_pub_;

  // ros::Publisher vel_pub_, loa_pub_;
  // ros::Subscriber joy_sub_;
  // ros::Timer timer_;
};

// Constructor

TeleopMobileRobot::TeleopMobileRobot() : Node("teleop_mobile_robot_node")

{

  // declare parameters and defaults
  // movement axis parameters (joystick axis maping)
  this->declare_parameter("axis_linear", 1);
  this->declare_parameter("axis_angular", 0);

  // commands scaling parameters
  this->declare_parameter("scaling_linear", 0.6);
  this->declare_parameter("scaling_angular", 0.4);

  // Joystick buttons mapping (default is for Xbox joystick)
  this->declare_parameter("teleop_button", 3);         // Y button for teleop LoA
  this->declare_parameter("auto_button", 0);           // A button for Autonomy LoA
  this->declare_parameter("shared_control_button", 2); // X button for Shared control LoA
  this->declare_parameter("stop_button", 4);           // LB button to stop everything

  // Get the parameters objects
  param_axis_linear_ = this->get_parameter("axis_linear");
  param_axis_angular_ = this->get_parameter("axis_angular");
  param_scaling_linear_ = this->get_parameter("scaling_linear");
  param_scaling_angular_ = this->get_parameter("scaling_angular");
  param_teleop_button_ = this->get_parameter("teleop_button");
  param_auto_button_ = this->get_parameter("auto_button");
  param_shared_control_button_ = this->get_parameter("shared_control_button");
  param_stop_button_ = this->get_parameter("stop_button");

  // Setup/create Subscriptions and Publishers
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&TeleopMobileRobot::joyCallback, this, std::placeholders::_1));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/teleop/cmd_vel", 10);
  loa_code_pub_ = this->create_publisher<std_msgs::msg::Int16>("/loa/code", 10);
  loa_name_pub_ = this->create_publisher<std_msgs::msg::String>("/loa/name", 10);
}

// Functions and CallBacks

void TeleopMobileRobot::joyCallback(const sensor_msgs::msg::Joy &joy_msg) const
{

  geometry_msgs::msg::Twist cmd_vel; // the teleoperation velocity commands to be published
  std_msgs::msg::Int16 loa_code;     // The LoA as a code/number
  std_msgs::msg::String loa_name;    // The LoA as a string/name

  // Movement commands
  cmd_vel.linear.x = param_scaling_linear_.as_double() * joy_msg.axes[param_axis_linear_.as_int()];
  cmd_vel.angular.z = param_scaling_angular_.as_double() * joy_msg.axes[param_axis_angular_.as_int()];
  cmd_vel_pub_->publish(cmd_vel);
  // last_msg_published_ = cmd_vel;

  // LoA switching
  if (joy_msg.buttons[param_stop_button_.as_int()])
  {
    loa_code.data = 0;
    loa_name.data = "Stop";
    loa_code_pub_->publish(loa_code);
    loa_name_pub_->publish(loa_name);
  }

  if (joy_msg.buttons[param_teleop_button_.as_int()])
  {
    loa_code.data = 1;
    loa_name.data = "Teleoperation";
    loa_code_pub_->publish(loa_code);
    loa_name_pub_->publish(loa_name);
  }

  if (joy_msg.buttons[param_auto_button_.as_int()])
  {
    loa_code.data = 2;
    loa_name.data = "Autonomy";
    loa_code_pub_->publish(loa_code);
    loa_name_pub_->publish(loa_name);
  }

  if (joy_msg.buttons[param_shared_control_button_.as_int()])
  {
    loa_code.data = 3;
    loa_name.data = "Shared_Control";
    loa_code_pub_->publish(loa_code);
    loa_name_pub_->publish(loa_name);
  }
}

int main(int argc, char *argv[])
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeleopMobileRobot>());
  rclcpp::shutdown();
  return 0;
}