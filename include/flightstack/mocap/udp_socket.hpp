/**************************************************************************
 * File:        udp_socket.hpp
 * Author:      girimugundankumar
 * Email:       girimugundan@vt.edu
 * Date:        April 21, 2024
 * 
 * Description: Class declaration for UDP socket creation using IoContext.
 * 
 * GitHub:      https://github.com/girimugundankumar/acsl-flight-stack.git
 * Referenced:  https://github.com/ros-drivers/transport_drivers/tree/main
 **************************************************************************/

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

	/*
	* Blocking Send Operation
	*/
	std::size_t send(std::vector<uint8_t> & buff);

	/*
	* Blocking Receive Operation
	*/
	size_t receive(std::vector<uint8_t> & buff);

	/*
	* NonBlocking Send Operation
	*/
	void asyncSend(std::vector<uint8_t> & buff);

	/*
	* NonBlocking Receive Operation
	*/
	void asyncReceive(Functor func);

private:
	void asyncSendHandler(
		const asio::error_code & error,
		std::size_t bytes_transferred);

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