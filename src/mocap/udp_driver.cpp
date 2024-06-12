/**************************************************************************
 * File:        udp_driver.cpp
 * Author:      girimugundankumar
 * Email:       girimugundan@vt.edu
 * Date:        April 21, 2024
 * 
 * Description: Class definition for UDP driver.
 * 
 * GitHub:      
 * Referenced:  https://github.com/ros-drivers/transport_drivers/tree/main
 **************************************************************************/

#include "udp_driver.hpp"

namespace _drivers_
{
namespace _udp_driver_
{

UdpDriver::UdpDriver(const IoContext & ctx)
: m_ctx(ctx)
{
}

void UdpDriver::init_sender(const std::string & ip, uint16_t port)
{
  m_sender = std::make_shared<UdpSocket>(m_ctx, ip, port);
}

void UdpDriver::init_sender(
  const std::string & remote_ip, uint16_t remote_port,
  const std::string & host_ip, uint16_t host_port)
{
  m_sender = std::make_shared<UdpSocket>(m_ctx, remote_ip, remote_port, host_ip, host_port);
}

void UdpDriver::init_receiver(const std::string & ip, uint16_t port)
{
  try {
      m_receiver = std::make_shared<UdpSocket>(m_ctx, ip, port);
  } catch (const std::invalid_argument& e) {
      // Handle the invalid argument exception
      std::cerr << "Invalid argument when initializing m_receiver: " << e.what() << std::endl;
      // Optionally, perform cleanup or take appropriate action
  } catch (const std::exception& e) {
      // Catch any other exceptions
      std::cerr << "Failed to initialize m_receiver: " << e.what() << std::endl;
      // Optionally, perform cleanup or take appropriate action
  }
}

std::shared_ptr<UdpSocket> UdpDriver::sender() const
{
  return m_sender;
}

std::shared_ptr<UdpSocket> UdpDriver::receiver() const
{
  return m_receiver;
}

}   // namespace _udp_driver_
}   // namespace _drivers_


