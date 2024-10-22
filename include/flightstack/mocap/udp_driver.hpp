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
 * File:        udp_driver.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        April 21, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Class for UDP driver.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack.git
 **********************************************************************************************************************/

/**
 * @file udp_driver.hpp
 * @brief Class for udp driver
 */
#ifndef UDP_DRIVER_HPP_
#define UDP_DRIVER_HPP_

#include <iostream>
#include <memory>
#include <string>

#include "udp_socket.hpp"

namespace _drivers_
{
namespace _udp_driver_
{

/**
 * @class UdpDriver
 * @brief UdpDriver class
 */
class UdpDriver
{
public:

	/**
	 * @brief Construct a new Udp Driver object
	 * 
	 * @param ctx 
	 */
	explicit UdpDriver(const IoContext & ctx);

	/**
	 * @brief init_sender ip and port
	 * @param ip 
	 * @param port 
	 */
	void init_sender(const std::string & ip, uint16_t port);

	/**
	 * @brief init_sender remote and host
	 * @param remote_ip 
	 * @param remote_port 
	 * @param host_ip 
	 * @param host_port 
	 */
	void init_sender(
		const std::string & remote_ip, uint16_t remote_port,
		const std::string & host_ip, uint16_t host_port);
	
	/**
	 * @brief init_receiver
	 * @param ip 
	 * @param port 
	 */
	void init_receiver(const std::string & ip, uint16_t port);

	std::shared_ptr<UdpSocket> sender() const;
	std::shared_ptr<UdpSocket> receiver() const;

private:
	const IoContext & m_ctx;
	std::shared_ptr<UdpSocket> m_sender;
	std::shared_ptr<UdpSocket> m_receiver;
};

}   // namespace _udp_driver_
}   // namespace _drivers_

#endif  // UDP_DRIVER_HPP_