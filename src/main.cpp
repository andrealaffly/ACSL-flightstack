/***********************************************************************************************************************
 * Copyright (c) 2024 Mattia Gramuglia, Giri M. Kumar, Andrea L'Afflitto. All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 *    disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *    following disclaimer in the documentation and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * File:        main.cpp
 * Author:      Mattia Gramuglia
 * Date:        April 9, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Main file
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack.git
 **********************************************************************************************************************/

#include "main.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  if constexpr (CURRENT_EKF2_FUSION_MODE == EKF2FusionMode::GPS)
  {
    // You MUST use the MultiThreadedExecutor to use, well, multiple threads
    rclcpp::executors::MultiThreadedExecutor executor;
    rclcpp::NodeOptions options;
    auto multi_threaded_node = std::make_shared<MultiThreadedNode>(options);
    executor.add_node(multi_threaded_node);

    std::cout << "CURRENT_EKF2_FUSION_MODE = GPS" << std::endl;

    executor.spin();
    rclcpp::shutdown();

  } else if constexpr (CURRENT_EKF2_FUSION_MODE == EKF2FusionMode::MOCAP)
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
