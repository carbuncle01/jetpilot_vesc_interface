#ifndef JETPILOT_VESC_INTERFACE__CONTROL_CMD_TO_VESC_NODE_HPP_
#define JETPILOT_VESC_INTERFACE__CONTROL_CMD_TO_VESC_NODE_HPP_

#include "jetpilot_msgs/msg/control_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace jetpilot_vesc_interface
{

class ControlCmdToVescNode : public rclcpp::Node
{
public:
  ControlCmdToVescNode();

private:
  void publish_latest_command();
  double normalized_drive_to_erpm(double throttle, double reverse) const;
  double normalized_steering_to_servo(double steering) const;
  static double apply_deadband(double value, double deadband);
  void publish_speed_and_servo(double speed_erpm, double servo_position);
  void publish_brake_and_servo(double brake_current_amps, double servo_position);

  double max_forward_erpm_;
  double max_reverse_erpm_;
  double max_brake_current_amps_;
  double servo_gain_;
  double servo_offset_;
  double servo_min_;
  double servo_max_;
  double throttle_deadband_;
  double reverse_deadband_;
  double brake_deadband_;
  double max_command_age_s_;

  rclcpp::Subscription<jetpilot_msgs::msg::ControlCommand>::SharedPtr cmd_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr brake_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr servo_publisher_;
  rclcpp::TimerBase::SharedPtr output_timer_;
  rclcpp::Time last_cmd_time_;
  jetpilot_msgs::msg::ControlCommand latest_cmd_;
  bool has_latest_cmd_{false};
};

}  // namespace jetpilot_vesc_interface

#endif  // JETPILOT_VESC_INTERFACE__CONTROL_CMD_TO_VESC_NODE_HPP_
