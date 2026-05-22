#include <memory>

#include "pendulum_safety/supervisor_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pendulum_safety::SafetySupervisor>());
  rclcpp::shutdown();
  return 0;
}
