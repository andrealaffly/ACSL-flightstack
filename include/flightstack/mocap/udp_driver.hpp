/**************************************************************************
 * File:        udp_driver.hpp
 * Author:      girimugundankumar
 * Email:       girimugundan@vt.edu
 * Date:        April 21, 2024
 * 
 * Description: Class declaration for UDP driver.
 * 
 * GitHub:      
 * Referenced:  https://github.com/ros-drivers/transport_drivers/tree/main
 **************************************************************************/

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

class UdpDriver
{
public:
	explicit UdpDriver(const IoContext & ctx);

	void init_sender(const std::string & ip, uint16_t port);
	void init_sender(
		const std::string & remote_ip, uint16_t remote_port,
		const std::string & host_ip, uint16_t host_port);
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