/**************************************************************************
 * File:        io_context.hpp
 * Author:      girimugundankumar
 * Email:       girimugundan@vt.edu
 * Date:        April 21, 2024
 * 
 * Description: Class declaration for IoContext to manage thread for udp.
 * 
 * GitHub:      
 * Referenced:  https://github.com/ros-drivers/transport_drivers/tree/main
 **************************************************************************/

#ifndef IO_CONTEXT_HPP_
#define IO_CONTEXT_HPP_

#include <memory>
#include <vector>
#include <utility>
#include <asio.hpp>
#include <iostream>

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