#include "rclcpp/rclcpp.hpp"
#include "can_transceiver/can_transceiver.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<CanTransceiver>(options));
  
  rclcpp::shutdown();
  return 0;
}