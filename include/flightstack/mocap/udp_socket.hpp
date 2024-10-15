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
 * File:        udp_socket.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        April 21, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Class for UDP socket creation using IoContext.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack.git
 **********************************************************************************************************************/

/**
 * @file udp_socket.hpp
 * @brief Class for UDP socket creation using IoContext
 */
#ifndef UDP_SOCKET_HPP_
#define UDP_SOCKET_HPP_

#include <array>
#include <string>
#include <vector>
#include <iostream>
#include <utility>
#include <system_error>

#include "io_context.hpp"

using asio::ip::udp;
using asio::ip::address;
using _drivers_::_common_::IoContext;

namespace _drivers_
{
namespace _udp_driver_
{

using Functor = std::function<void (const std::vector<uint8_t> &)>;

/**
 * @class UdpSocket
 */
class UdpSocket
{
public:
	UdpSocket(
		const IoContext & ctx,
		const std::string & remote_ip, uint16_t remote_port,
		const std::string & host_ip, uint16_t host_port);

	UdpSocket(
		const IoContext & ctx,
		const std::string & ip, uint16_t port);

	~UdpSocket();

	UdpSocket(const UdpSocket &) = delete;
	UdpSocket & operator=(const UdpSocket &) = delete;

	std::string remote_ip() const;
	uint16_t remote_port() const;
	std::string host_ip() const;
	uint16_t host_port() const;

	void open();
	void close();
	bool isOpen() const;
	void bind();

	/**
	 * Blocking Send Operation
	 */
	std::size_t send(std::vector<uint8_t> & buff);

	/**
	 * Blocking Receive Operation
	 */
	size_t receive(std::vector<uint8_t> & buff);

	/**
	 * NonBlocking Send Operation
	 * @param buff 
	 */
	void asyncSend(std::vector<uint8_t> & buff);

	/**
	 * NonBlocking Receive Operation
	 * @param func 
	 */
	void asyncReceive(Functor func);

private:
	/**
	 * asyncSendHandler
	 * @param error 
	 * @param bytes_transferred 
	 */
	void asyncSendHandler(
		const asio::error_code & error,
		std::size_t bytes_transferred);

	/**
	 * asyncReceiveHandler
	 * @param error 
	 * @param bytes_transferred 
	 */
	void asyncReceiveHandler(
		const asio::error_code & error,
		std::size_t bytes_transferred);

private:
	const IoContext & m_ctx;
	udp::socket m_udp_socket;
	udp::endpoint m_remote_endpoint;
	udp::endpoint m_host_endpoint;
	Functor m_func;
	static const size_t m_recv_buffer_size{255}; // To match the vicon system
	std::vector<uint8_t> m_recv_buffer;
};

}   // namespace _udp_driver
}   // namespace _drivers_


#endif  // UDP_SOCKET_HPP_