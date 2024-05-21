/**************************************************************************
 * File:        udp_socket.cpp
 * Author:      girimugundankumar
 * Email:       girimugundan@vt.edu
 * Date:        April 21, 2024
 * 
 * Description: Class definition for UDP socket creation using IoContext.
 * 
 * GitHub:      https://github.com/girimugundankumar/acsl-flight-stack.git
 * Referenced:  https://github.com/ros-drivers/transport_drivers/tree/main
 **************************************************************************/

#include "udp_socket.hpp"

namespace _drivers_
{
namespace _udp_driver_
{

UdpSocket::UdpSocket(
  const IoContext & ctx,
  const std::string & remote_ip,
  const uint16_t remote_port,
  const std::string & host_ip,
  const uint16_t host_port)
: m_ctx(ctx),
  m_udp_socket(ctx.ios()),
  m_remote_endpoint(address::from_string(remote_ip), remote_port),
  m_host_endpoint(address::from_string(host_ip), host_port)
{
  m_remote_endpoint = remote_ip.empty() ?
    udp::endpoint{udp::v4(), remote_port} :
  udp::endpoint{address::from_string(remote_ip), remote_port};
  m_host_endpoint = host_ip.empty() ?
    udp::endpoint{udp::v4(), host_port} :
  udp::endpoint{address::from_string(host_ip), host_port};
  m_recv_buffer.resize(m_recv_buffer_size);
}

UdpSocket::UdpSocket(
  const IoContext & ctx,
  const std::string & ip,
  const uint16_t port)
: UdpSocket{ctx, ip, port, ip, port}
{}

UdpSocket::~UdpSocket()
{
  close();
}

std::size_t UdpSocket::send(std::vector<uint8_t> & buff)
{
  try {
    return m_udp_socket.send_to(asio::buffer(buff), m_remote_endpoint);
  } catch (const std::system_error & error) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("UdpSocket::send"), error.what());
    return -1;
  }
}

size_t UdpSocket::receive(std::vector<uint8_t> & buff)
{
  asio::error_code error;
  asio::ip::udp::endpoint sender_endpoint;

  std::size_t len = m_udp_socket.receive_from(
    asio::buffer(buff),
    m_host_endpoint,
    0,
    error);

  if (error && error != asio::error::message_size) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("UdpSocket::receive"), error.message());
    return -1;
  }
  return len;
}

void UdpSocket::asyncSend(std::vector<uint8_t> & buff)
{
  m_udp_socket.async_send_to(
    asio::buffer(buff), m_remote_endpoint,
    [this](std::error_code error, std::size_t bytes_transferred)
    {
      asyncSendHandler(error, bytes_transferred);
    });
}

void UdpSocket::asyncReceive(Functor func)
{
  m_func = std::move(func);
  m_udp_socket.async_receive_from(
    asio::buffer(m_recv_buffer),
    m_host_endpoint,
    [this](std::error_code error, std::size_t bytes_transferred)
    {
      asyncReceiveHandler(error, bytes_transferred);
    });
}

void UdpSocket::asyncSendHandler(
  const asio::error_code & error,
  std::size_t bytes_transferred)
{
  (void)bytes_transferred;
  if (error) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("UdpSocket::asyncSendHandler"), error.message());
  }
}

void UdpSocket::asyncReceiveHandler(
  const asio::error_code & error,
  std::size_t bytes_transferred)
{
  (void)bytes_transferred;
  if (error) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("UdpSocket::asyncReceiveHandler"), error.message());
    std::exit(EXIT_FAILURE);
    return;
  }

  if (bytes_transferred > 0 && m_func) {
    m_recv_buffer.resize(bytes_transferred);
    m_func(m_recv_buffer);
    m_recv_buffer.resize(m_recv_buffer_size);
    m_udp_socket.async_receive_from(
      asio::buffer(m_recv_buffer),
      m_host_endpoint,
      [this](std::error_code error, std::size_t bytes_tf)
      {
        m_recv_buffer.resize(bytes_tf);
        asyncReceiveHandler(error, bytes_tf);
      });
  }
}

std::string UdpSocket::remote_ip() const
{
  return m_remote_endpoint.address().to_string();
}

uint16_t UdpSocket::remote_port() const
{
  return m_remote_endpoint.port();
}

std::string UdpSocket::host_ip() const
{
  return m_host_endpoint.address().to_string();
}

uint16_t UdpSocket::host_port() const
{
  return m_host_endpoint.port();
}

void UdpSocket::open()
{
  try {
    m_udp_socket.open(udp::v4());
    m_udp_socket.set_option(udp::socket::reuse_address(true));
    std::cout << "Socket receive is open" << std::endl;
  } catch (const std::exception& e) {
      // Handle the exception
      std::cerr << "An error occurred while opening or setting options for m_udp_socket: " << e.what() << std::endl;
      std::exit(EXIT_FAILURE);
      // Optionally, perform cleanup or take appropriate action
  }
}

void UdpSocket::close()
{
  asio::error_code error;
  m_udp_socket.close(error);
  if (error) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("UdpSocket::close"), error.message());
  }
}

bool UdpSocket::isOpen() const
{
  return m_udp_socket.is_open();
}

void UdpSocket::bind()
{
  try {
    m_udp_socket.bind(m_host_endpoint);
    std::cout << "Socket receive is bound" << std::endl;
  } catch (const std::exception& e) {
      // Handle the exception
      std::cerr << "An error occurred while binding m_udp_socket: " << e.what() << std::endl;
      std::exit(EXIT_FAILURE);
      // Optionally, perform cleanup or take appropriate action
  }
}

}   // namespace _udp_driver_
}   // namespace _drivers_