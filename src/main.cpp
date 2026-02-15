#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "hexapod_sim/gait_controller_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hexapod_sim::GaitControllerNode>();
  const bool ok = node->run_startup_sequence();
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
