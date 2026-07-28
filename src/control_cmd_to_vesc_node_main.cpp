#include <memory>

#include "jetpilot_vesc_interface/control_cmd_to_vesc_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<jetpilot_vesc_interface::ControlCmdToVescNode>());
  rclcpp::shutdown();
  return 0;
}
