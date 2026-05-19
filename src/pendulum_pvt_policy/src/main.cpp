#include <rclcpp/rclcpp.hpp>

#include "pendulum_pvt_policy/policy_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pendulum_pvt_policy::PvtPolicyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
