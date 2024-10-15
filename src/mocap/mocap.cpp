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
 * File:        mocap.cpp
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

#include "mocap.hpp"
/**
 * @file mocap.cpp
 * @brief Node definition for UDP socket as a lifecycle node.
 * Writes messages to pixhawk mocap_odometry topic for EFK2 fusion.
 */

namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using lifecycle_msgs::msg::State;

namespace _drivers_
{
namespace _udp_driver_
{
  

/**
 * @brief Construct a new Udp Receiver Node:: Udp Receiver Node object
 * 
 * @param[in] ctx 
 * @param timestamp_initial 
 */
UdpReceiverNode::UdpReceiverNode(
  const IoContext & ctx,
  const std::atomic<uint64_t>& timestamp_initial)
: lc::LifecycleNode("udp_receiver_node"),
  m_udp_driver{std::make_unique<UdpDriver>(ctx)},
  timestamp_initial_(timestamp_initial) 
{
  get_params();

  mocap_data_ = std::make_shared<MocapData>(*this);

  // Initialize Mocap logging
	mocap_data_->logInitializeLogging();
}


/**
 * @brief Get the parameters for the ip and port to ping
 * @param None
 */

void UdpReceiverNode::get_params()
{
  // m_ip = GROUND_STATION_IP; // For testing with internal port on groundstation
  m_ip = ODROID_M1S_IP;     // For implementation with VICON and ODroid M1s

  m_port = ODROID_M1S_PORT;

  RCLCPP_INFO(get_logger(), "ip: %s", m_ip.c_str());
  RCLCPP_INFO(get_logger(), "port: %i", m_port);
}

/// \brief Destructor - required to manage owned IoContext
UdpReceiverNode::~UdpReceiverNode()
{
  if (m_owned_ctx) {
    m_owned_ctx->waitForExit();
  }
}

// Check that Lifecycle Node Starts
/**
 * @fn UdpReceiverNode::checkLifecycleNodeStarted()
 * @brief Check that Lifecycle Node Starts
 * @param None
 */
void UdpReceiverNode::checkLifecycleNodeStarted()
{
  if (this->configure().id() == State::PRIMARY_STATE_INACTIVE) {
      if (this->activate().id() == State::PRIMARY_STATE_ACTIVE) {
      } else {
      throw std::runtime_error{"Failed to activate UDP receiver."};
      }
  } else {
      throw std::runtime_error{"Failed to configure UDP receiver."};
  }
}
.

/**
 * @brief Callbackj from transition to "configuration state"
 * 
 * @param state The current state that the node is in
 * @return LNI::CallbackReturn 
 */

LNI::CallbackReturn UdpReceiverNode::on_configure(const lc::State & state)
{
  (void)state;

  // Check if m_udp_driver is not null
  if (m_udp_driver) {
    RCLCPP_INFO(get_logger(), "UDP driver is initialized.");
  }

  mocap_publisher_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
    "/fmu/in/vehicle_visual_odometry", rclcpp::QoS(100));

  try {
    
    m_udp_driver->init_receiver(m_ip, m_port);
    m_udp_driver->receiver()->open();
    m_udp_driver->receiver()->bind();
    m_udp_driver->receiver()->asyncReceive(
      std::bind(&UdpReceiverNode::receiver_callback, this, std::placeholders::_1));
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating UDP receiver: %s:%i - %s",
      m_ip.c_str(), m_port, ex.what());
    return LNI::CallbackReturn::FAILURE;
  }

  RCLCPP_DEBUG(get_logger(), "UDP receiver successfully configured.");

  return LNI::CallbackReturn::SUCCESS;
}

/// \brief Callback from transition to "activating" state.
/// \param[in] state The current state that the node is in.

/**
 * @brief Callback from transition to "acitvating" state.
 * 
 * @param state 
 * @return LNI::CallbackReturn 
 */

LNI::CallbackReturn UdpReceiverNode::on_activate(const lc::State & state)
{
  (void)state;
  mocap_publisher_->on_activate();
  RCLCPP_DEBUG(get_logger(), "UDP receiver activated.");
  return LNI::CallbackReturn::SUCCESS;
}


/// \brief Callback from transition to "deactivating" state.
/// \param[in] state The current state that the node is in.
LNI::CallbackReturn UdpReceiverNode::on_deactivate(const lc::State & state)
{
  (void)state;
  mocap_publisher_->on_deactivate();
  RCLCPP_DEBUG(get_logger(), "UDP receiver deactivated.");
  return LNI::CallbackReturn::SUCCESS;
}


/// \brief Callback from transition to "unconfigured" state.
/// \param[in] state The current state that the node is in.

/**
 * @brief Callback from transition to "unconfigured" state.
 * 
 * @param state 
 * @return LNI::CallbackReturn 
 */

LNI::CallbackReturn UdpReceiverNode::on_cleanup(const lc::State & state)
{
  (void)state;
  m_udp_driver->receiver()->close();
  mocap_publisher_.reset();
  RCLCPP_DEBUG(get_logger(), "UDP receiver cleaned up.");
  return LNI::CallbackReturn::SUCCESS;
}

/// \brief Callback from transition to "shutdown" state.
/// \param[in] state The current state that the node is in.
LNI::CallbackReturn UdpReceiverNode::on_shutdown(const lc::State & state)
{
  (void)state;
  RCLCPP_DEBUG(get_logger(), "UDP receiver shutting down.");
  return LNI::CallbackReturn::SUCCESS;
}

// Getter function for the mocap_states
mocap_states& UdpReceiverNode::getMocapStates() {
  return mc;
}

// Getter function for the timestamp_mocap_
uint64_t& UdpReceiverNode::getTimestampMocap() {
  return timestamp_mocap_;
}

// Getter function for the time_mocap_
double& UdpReceiverNode::getTimeMocap() {
  return time_mocap_;
}

/*
 A small convenience function for converting a thread ID to a string
*/
std::string string_thread_id()
{
  auto hashed = std::hash<std::thread::id>()(std::this_thread::get_id());
  return std::to_string(hashed);
}

/// \brief Debugger function to output the mocap data.
void UdpReceiverNode::debugMocapData2screen()
{
  // Output the parsed data
  // std::cout << "===== MOCAP DATA =====" << std::endl;

  /* // Extract current thread
  auto thread_string = "THREAD " + string_thread_id();
	std::cout << "\n\n";
  std::cout << "MOCAP " << thread_string << std::endl;
  */

  timestamp_mocap_ = this->get_clock()->now().nanoseconds() / 1000;
  // std::cout << "Timestamp: " << timestamp_mocap_ << std::endl;

  time_mocap_ = (timestamp_mocap_ - timestamp_initial_) / 1e6;
  // std::cout << "Time MoCap message [s]: " <<  time_mocap_ << std::endl;

	/* 
  std::cout << "Position (x, y, z): " << mc.x << ", " 
                                      << mc.y << ", "
                                      << mc.z << std::endl;
  std::cout << "Quaternion (q0, q1, q2, q3): " << mc.q0 << ", "
                                               << mc.q1 << ", "
                                               << mc.q2 << ", "
                                               << mc.q3 << std::endl;
  std::cout << "Velocity (vx, vy, vz): " << mc.vx << ", "
                                         << mc.vy << ", " 
                                         << mc.vz << std::endl;
  std::cout << "Angular velocities (rollspeed, pitchspeed, yawspeed): " << mc.rollspeed << ", " 
                                                                        << mc.pitchspeed << ","
                                                                        << mc.yawspeed << std::endl;  
  */

  mocap_data_->logMocapData();

}

/// \brief Callback for receiving a UDP datagram
void UdpReceiverNode::receiver_callback(const std::vector<uint8_t> & buffer)
{
  // Convert the buffer to a string
  std::string buffer_str(buffer.begin(), buffer.end());

  // Check if the message format is valid
  if (buffer_str.size() < 2 || buffer_str[0] != 'R' || buffer_str[1] != ',') {
      std::cerr << "Invalid message format!" << std::endl;
      return; // Exit the function or handle the error accordingly
  }


  // Parse the data from the string
  std::istringstream iss(buffer_str.substr(2)); // Skip "R," prefix
  char comma;
  iss >> mc.x >> comma >> mc.y >> comma >> mc.z >> comma                       // Read in the position 
      >> mc.q0 >> comma >> mc.q1 >> comma >> mc.q2 >> comma >> mc.q3 >> comma  // Read in the quaternion
      >> mc.vx >> comma >> mc.vy >> comma >> mc.vz >> comma                    // Read in the velocities
      >> mc.rollspeed >> comma >> mc.pitchspeed >> comma >> mc.yawspeed;       // Read in the angular speeds

  // Debug Mocap parsed data to terminal
  debugMocapData2screen();

  // Initilize the Mocap odometry message to publish to the pixhawk
  px4_msgs::msg::VehicleOdometry out;

  // \brief: https://github.com/hpaul360/mocap_to_px4/blob/main/mocap_to_px4/converter_member_function.py
  // Package the message   
  out.timestamp = 0;        // Timestamp is automatically set inside PX4
  out.timestamp_sample = 0; // Timestamp is automatically set inside PX4

  // Position frame is set to NED in the VICON code.
  out.position[0] = (float) mc.x;
  out.position[1] = (float) mc.y;
  out.position[2] = (float) mc.z;  
  // Orientation frame is set to NED in the VICON code.
  out.q[0] = (float) mc.q0;
  out.q[1] = (float) mc.q1;
  out.q[2] = (float) mc.q2;
  out.q[3] = (float) mc.q3;
  out.pose_frame = POSE_FRAME_NED;

  // Velocity frame is set to NED in the VICON code. 
  out.velocity[0] = (float) mc.vx;
  out.velocity[1] = (float) mc.vy;
  out.velocity[2] = (float) mc.vz;
  out.velocity_frame = VELOCITY_FRAME_NED;

  // Angular speed is set to 0. We need to pass the velocites to EKF2 in the FRD Body-Fixed
  // Frame. We do not have the data in the VICON code therefore we set it to 0.
  out.angular_velocity[0] = 0;
  out.angular_velocity[1] = 0;
  out.angular_velocity[2] = 0;

  // Other Parameters
  out.position_variance = {0,0,0};
  out.orientation_variance = {0,0,0};
  out.velocity_variance = {0,0,0};

  // Publish the message
  mocap_publisher_->publish(out);

}


} // namespace _udp_driver_    
} // namespace _drivers_

