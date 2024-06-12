/**************************************************************************
 * File:        io_context.cpp
 * Author:      girimugundankumar
 * Email:       girimugundan@vt.edu
 * Date:        April 21, 2024
 * 
 * Description: Class definition for IoContext to manage thread for udp.
 * 
 * GitHub:      
 * Referenced:  https://github.com/ros-drivers/transport_drivers/tree/main
 **************************************************************************/

#include "io_context.hpp"

namespace _drivers_
{
namespace _common_
{

// Delegating to the modified constructor with -1 for default CPU
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