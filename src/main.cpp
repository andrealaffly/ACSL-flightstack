/* main.cpp

Mattia Gramuglia
April 9, 2024
*/

#include "main.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  if (CURRENT_EKF2_FUSION_MODE == EKF2FusionMode::GPS)
  {
    // You MUST use the MultiThreadedExecutor to use, well, multiple threads
    rclcpp::executors::MultiThreadedExecutor executor;
    rclcpp::NodeOptions options;
    auto multi_threaded_node = std::make_shared<MultiThreadedNode>(options);
    executor.add_node(multi_threaded_node);

    std::cout << "CURRENT_EKF2_FUSION_MODE = GPS" << std::endl;

    executor.spin();
    rclcpp::shutdown();

  } else if (CURRENT_EKF2_FUSION_MODE == EKF2FusionMode::MOCAP)
  {
    // You MUST use the MultiThreadedExecutor to use, well, multiple threads
    rclcpp::executors::MultiThreadedExecutor executor;
    rclcpp::NodeOptions options;
    auto multi_threaded_node = std::make_shared<MultiThreadedNode>(options);
    executor.add_node(multi_threaded_node);

    std::cout << "CURRENT_EKF2_FUSION_MODE = MOCAP" << std::endl;
    _drivers_::_common_::IoContext ctx{1}; 
    auto mocap_node = std::make_shared<_drivers_::_udp_driver_::UdpReceiverNode>(
      ctx,
      multi_threaded_node->getInitialTimestamp());
    executor.add_node(mocap_node->get_node_base_interface());
    mocap_node->checkLifecycleNodeStarted();

    executor.spin();
    rclcpp::shutdown();
  }

  return 0;
}
