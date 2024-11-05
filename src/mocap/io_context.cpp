///@cond
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
///@endcond
/***********************************************************************************************************************
 * File:        io_context.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        April 21, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Class for IoContext to manage thread for udp.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack.git
 **********************************************************************************************************************/

#include "io_context.hpp"
/**
 * @file io_context.cpp
 * @brief Class for IoContext to manage thread for udp
 */

namespace _drivers_
{
namespace _common_
{

// Delegating to the modified constructor with -1 for default CPU

/**
 * @class IoContext
 * Delegating to the modified constructor with -1 for default CPU
 */
IoContext::IoContext()
: IoContext(std::thread::hardware_concurrency()) {}

IoContext::IoContext(size_t threads_count)
: m_ios(std::make_shared<asio::io_service>()),
  m_work(std::make_shared<asio::io_service::work>(ios())),
  m_ios_thread_workers(std::make_shared<_drivers_::_common_::thread_group>())
{
  for (size_t i = 0; i < threads_count; ++i) {
    m_ios_thread_workers->create_thread(
      [this]() {
        ios().run(); 
      });
  }

  RCLCPP_INFO_STREAM(rclcpp::get_logger("IoContext::IoContext"),
   "Thread(s) Created: " << serviceThreadCount());

}

IoContext::~IoContext()
{
  waitForExit();
}

asio::io_service & IoContext::ios() const
{
  return *m_ios;
}

bool IoContext::isServiceStopped()
{
  return ios().stopped();
}

uint32_t IoContext::serviceThreadCount()
{
  return m_ios_thread_workers->size();
}

void IoContext::waitForExit()
{
  if (!ios().stopped()) {
    ios().post([&]() {m_work.reset();});
  }

  ios().stop();
  m_ios_thread_workers->join_all();
}

}   // namespace _common_
}   // namespace _drivers_