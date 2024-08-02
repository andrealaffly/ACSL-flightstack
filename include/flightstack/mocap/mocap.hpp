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
 * Part of the code in this file leverages the following material.
 *
 * Referenced:  https://github.com/ros-drivers/transport_drivers/tree/main
 *              Copyright 2021 LeoDrive.
 *            
 *              Licensed under the Apache License, Version 2.0 (the "License");
 *              you may not use this file except in compliance with the License.
 *              You may obtain a copy of the License at
 *               
 *                  http://www.apache.org/licenses/LICENSE-2.0
 *              
 *              Unless required by applicable law or agreed to in writing, software
 *              distributed under the License is distributed on an "AS IS" BASIS,
 *              WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *              See the License for the specific language governing permissions and
 *              limitations under the License.
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * File:        mocap.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        April 20, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Node definition for UDP socket as a lifecycle node.
 *              Writes messages to pixhawk mocap_odometry topic for 
 *              EFK2 fusion.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack.git
 **********************************************************************************************************************/

#ifndef MOCAP_HPP_
#define MOCAP_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <bit>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "lifecycle_msgs/msg/transition.hpp"
#include <lifecycle_msgs/msg/state.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include "udp_driver.hpp"
#include "logging_mocap.hpp"

using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

// -> IP of the Odroid M1s
inline constexpr const char* GROUND_STATION_IP = "127.0.0.1"; // For testing 
inline constexpr const char* ODROID_M1S_IP = "192.168.12.1";
// -> PORT of the Odroid M1s
inline constexpr uint16_t ODROID_M1S_PORT = 52000;


/// ------------- USER DEFINED ------------- ///
// -> Pixhawk vehicle odometry pose frame message package definitions
inline constexpr uint8_t POSE_FRAME_UNKNOWN = 0; // Pose frame unkown
inline constexpr uint8_t POSE_FRAME_NED     = 1; // NED earth-fixed frame
inline constexpr uint8_t POSE_FRAME_FRD     = 2; // FRD world-fixed frame, arbitrary heading reference
// -> Pixhawk vehicle odometry velocity framemessage pacakge definitions
inline constexpr uint8_t VELOCITY_FRAME_UNKNOWN = 0;  // Velocity frame unkown
inline constexpr uint8_t VELOCITY_FRAME_NED      = 1; // NED earth-fixed frame
inline constexpr uint8_t VELOCITY_FRAME_FRD      = 2; // FRD world-fixed frame, arbitrary heading reference
inline constexpr uint8_t VELOCITY_FRAME_BODY_FRD = 3; // FRD body-fixed frame

namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

namespace _drivers_
{
namespace _udp_driver_
{

/// Define struct variables to store parsed data
  struct mocap_states
  {
    double x; 
    double y;
    double z;
    double q0;
    double q1;
    double q2;
    double q3;
    double vx;
    double vy;
    double vz;
    double rollspeed;
    double pitchspeed;
    double yawspeed;
  };

/// \brief UdpReceiverNode class which can receive UDP datagrams
class UdpReceiverNode final
  : public lc::LifecycleNode
{
public:
  /// \brief Constructor which accepts IoContext
  /// \param[in] ctx A shared IoContext
  /// \param[in] pointer to vehicle states in flight_bridge
  UdpReceiverNode(const IoContext & ctx, const std::atomic<uint64_t>& timestamp_initial);

  /// \brief Destructor - required to manage owned IoContext
  ~UdpReceiverNode();

  void checkLifecycleNodeStarted();

  /// \brief Callback from transition to "configuring" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_configure(const lc::State & state) override;

  /// \brief Callback from transition to "activating" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_activate(const lc::State & state) override;

  /// \brief Callback from transition to "deactivating" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_deactivate(const lc::State & state) override;

  /// \brief Callback from transition to "unconfigured" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_cleanup(const lc::State & state) override;

  /// \brief Callback from transition to "shutdown" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_shutdown(const lc::State & state) override;

  /// \brief Callback for receiving a UDP datagram
  void receiver_callback(const std::vector<uint8_t> & buffer);

  // Getter function for the mocap_states
  mocap_states& getMocapStates();

  // Getter function for the timestamp_mocap_
  uint64_t& getTimestampMocap();

  // Getter function for the time_mocap_
  double& getTimeMocap();

private:

  /// \brief Get the parameters for the ip and port to ping
  void get_params();

  /// Pointer to the asio context owned by this node for async communication
  std::unique_ptr<IoContext> m_owned_ctx{};
  
  /// String for the ip of the odrioid
  std::string m_ip{};
      
  /// String for the port of the odroid
  uint16_t m_port{};
  
  /// Pointer for the udp driver which wraps the udp socket
  std::unique_ptr<UdpDriver> m_udp_driver;
  
  /// Publisher for publishing the mocap message
  lc::LifecyclePublisher<px4_msgs::msg::VehicleOdometry>::SharedPtr mocap_publisher_;

  // Reference to the initial timestamp determined in multi_threaded_node
  const std::atomic<uint64_t>& timestamp_initial_;

  // Timestamp at which the mocap data is received
  uint64_t timestamp_mocap_;

  // Time in seconds at which the mocap data is received wrt the timestamp_initial_
  double time_mocap_;

  /// Instantiate mocap_states struct
  struct mocap_states mc;

  /// \brief Debugger function to output the mocap data.
  void debugMocapData2screen();

  // Create a pointer to the MocapData instance
  std::shared_ptr<MocapData> mocap_data_;

};  

// DUPLICATE - already present in multi_threaded_node
std::string string_thread_id();

}   // namespace _udp_driver_
}   // namesapce _drivers_

#endif  // MOCAP_HPP_