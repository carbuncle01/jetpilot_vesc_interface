#include <algorithm>
#include <chrono>

#include "jetpilot_vesc_interface/control_cmd_to_vesc_node.hpp"

namespace jetpilot_vesc_interface
{

ControlCmdToVescNode::ControlCmdToVescNode() : Node("control_cmd_to_vesc_node")
{
  max_forward_erpm_ = declare_parameter<double>("max_forward_erpm", 14000.0);
  max_reverse_erpm_ = declare_parameter<double>("max_reverse_erpm", 5000.0);
  max_brake_current_amps_ = declare_parameter<double>("max_brake_current_amps", 20.0);
  servo_gain_ = declare_parameter<double>("servo_gain", -0.315);
  servo_offset_ = declare_parameter<double>("servo_offset", 0.5304);
  servo_min_ = declare_parameter<double>("servo_min", 0.15);
  servo_max_ = declare_parameter<double>("servo_max", 0.85);
  throttle_deadband_ = declare_parameter<double>("throttle_deadband", 0.03);
  reverse_deadband_ = declare_parameter<double>("reverse_deadband", 0.03);
  brake_deadband_ = declare_parameter<double>("brake_deadband", 0.03);
  max_command_age_s_ = declare_parameter<double>("max_command_age_s", 0.5);
  const auto output_rate_hz = std::max(1.0, declare_parameter<double>("output_rate_hz", 50.0));

  const auto qos_cmd = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  cmd_subscription_ = create_subscription<jetpilot_msgs::msg::ControlCommand>(
    "/control_cmd", qos_cmd,
    [this](jetpilot_msgs::msg::ControlCommand::SharedPtr msg)
    {
      latest_cmd_ = *msg;
      has_latest_cmd_ = true;
      last_cmd_time_ = now();
    });

  speed_publisher_ = create_publisher<std_msgs::msg::Float64>("commands/motor/speed", 10);
  brake_publisher_ = create_publisher<std_msgs::msg::Float64>("commands/motor/brake", 10);
  servo_publisher_ = create_publisher<std_msgs::msg::Float64>("commands/servo/position", 10);

  const auto output_period = std::chrono::duration<double>(1.0 / output_rate_hz);
  output_timer_ =
    create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(output_period),
                      [this]() { publish_latest_command(); });
}

void ControlCmdToVescNode::publish_latest_command()
{
  if (!has_latest_cmd_)
  {
    publish_speed_and_servo(0.0, servo_offset_);
    return;
  }

  if ((now() - last_cmd_time_).seconds() > std::max(0.0, max_command_age_s_))
  {
    publish_speed_and_servo(0.0, servo_offset_);
    return;
  }

  const auto steering = std::clamp(static_cast<double>(latest_cmd_.steering), -1.0, 1.0);
  const auto throttle = apply_deadband(
    std::clamp(static_cast<double>(latest_cmd_.throttle), 0.0, 1.0), throttle_deadband_);
  const auto reverse = apply_deadband(
    std::clamp(static_cast<double>(latest_cmd_.reverse), 0.0, 1.0), reverse_deadband_);
  const auto brake =
    apply_deadband(std::clamp(static_cast<double>(latest_cmd_.brake), 0.0, 1.0), brake_deadband_);

  const auto servo_position = normalized_steering_to_servo(steering);
  if (brake > 0.0)
  {
    publish_brake_and_servo(brake * std::max(0.0, max_brake_current_amps_), servo_position);
    return;
  }

  publish_speed_and_servo(normalized_drive_to_erpm(throttle, reverse), servo_position);
}

double ControlCmdToVescNode::normalized_drive_to_erpm(double throttle, double reverse) const
{
  if (throttle >= reverse)
  {
    return throttle * std::max(0.0, max_forward_erpm_);
  }
  return -reverse * std::max(0.0, max_reverse_erpm_);
}

double ControlCmdToVescNode::normalized_steering_to_servo(double steering) const
{
  const auto min_servo = std::min(servo_min_, servo_max_);
  const auto max_servo = std::max(servo_min_, servo_max_);
  return std::clamp(servo_offset_ + steering * servo_gain_, min_servo, max_servo);
}

double ControlCmdToVescNode::apply_deadband(double value, double deadband)
{
  return value > std::clamp(deadband, 0.0, 1.0) ? value : 0.0;
}

void ControlCmdToVescNode::publish_speed_and_servo(double speed_erpm, double servo_position)
{
  std_msgs::msg::Float64 speed_msg;
  speed_msg.data = speed_erpm;
  speed_publisher_->publish(speed_msg);

  std_msgs::msg::Float64 servo_msg;
  servo_msg.data = servo_position;
  servo_publisher_->publish(servo_msg);
}

void ControlCmdToVescNode::publish_brake_and_servo(double brake_current_amps, double servo_position)
{
  std_msgs::msg::Float64 speed_msg;
  speed_msg.data = 0.0;
  speed_publisher_->publish(speed_msg);

  std_msgs::msg::Float64 brake_msg;
  brake_msg.data = brake_current_amps;
  brake_publisher_->publish(brake_msg);

  std_msgs::msg::Float64 servo_msg;
  servo_msg.data = servo_position;
  servo_publisher_->publish(servo_msg);
}

}  // namespace jetpilot_vesc_interface
