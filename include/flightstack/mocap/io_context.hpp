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
 * File:        io_context.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        April 21, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Class for IoContext to manage thread for udp.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack.git
 **********************************************************************************************************************/

#ifndef IO_CONTEXT_HPP_
#define IO_CONTEXT_HPP_

#include <memory>
#include <vector>
#include <utility>
#include <asio.hpp>
#include <iostream>
#include <thread>

#include <rclcpp/logging.hpp>

namespace _drivers_
{
namespace _common_
{

///! A workaround of boost::thread_group
// Copied from https://gist.github.com/coin-au-carre/ceb8a790cec3b3535b015be3ec2a1ce2
struct thread_group
{
  std::vector<std::thread> tg;

  thread_group()                                  = default;
  thread_group(const thread_group &)               = delete;
  thread_group & operator=(const thread_group &)    = delete;
  thread_group(thread_group &&)                   = delete;

  template<class ... Args>
  void create_thread(Args && ... args) {tg.emplace_back(std::forward<Args>(args)...);}

  void add_thread(std::thread && t) {tg.emplace_back(std::move(t));}

  std::size_t size() const {return tg.size();}

  void join_all()
  {
    for (auto & thread : tg) {
      if (thread.joinable()) {
        thread.join();
      }
    }
  }

};

class IoContext
{
public:
  IoContext();
  explicit IoContext(size_t threads_count);
  ~IoContext();

  IoContext(const IoContext &) = delete;
  IoContext & operator=(const IoContext &) = delete;

  asio::io_service & ios() const;

  bool isServiceStopped();
  uint32_t serviceThreadCount();

  void waitForExit();

  template<class F>
  void post(F f)
  {
    ios().post(f);
  }

private:
  std::shared_ptr<asio::io_service> m_ios;
  std::shared_ptr<asio::io_service::work> m_work;
  std::shared_ptr<_drivers_::_common_::thread_group> m_ios_thread_workers;
};

}   // namespace _common_
}   // namespace _drivers_




#endif  // IO_CONTEXT_HPP_