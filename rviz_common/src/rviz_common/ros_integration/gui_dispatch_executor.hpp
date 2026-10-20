/*
 * Copyright (c) 2026, nevlife.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RVIZ_COMMON__ROS_INTEGRATION__GUI_DISPATCH_EXECUTOR_HPP_
#define RVIZ_COMMON__ROS_INTEGRATION__GUI_DISPATCH_EXECUTOR_HPP_

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

#include "rclcpp/executor.hpp"

#include "rviz_common/visibility_control.hpp"

namespace rviz_common
{
namespace ros_integration
{

// Takes messages, requests, responses, timer events and waitable data on a ROS thread, so that
// deserialization does not block rendering, and runs every callback in dispatch(), on the thread
// that calls it. dispatch() keeps the spin_some() contract: each entity runs at most once per
// call. A subscription keeps at most its KEEP_LAST depth of taken messages, oldest dropped first.
//
// A wait owns every entity in the wait set. dispatch() first moves the ROS thread out of its
// wait, so an entity whose owner released it is already gone and its callbacks are dropped,
// as with spin_some().
class RVIZ_COMMON_PUBLIC GuiDispatchExecutor : public rclcpp::Executor
{
public:
  explicit GuiDispatchExecutor(
    const rclcpp::ExecutorOptions & options = rclcpp::ExecutorOptions());
  ~GuiDispatchExecutor() override;

  void start();
  void stop();
  void dispatch(std::chrono::nanoseconds max_duration);
  size_t pending();

  void spin() override;

private:
  struct Item
  {
    uint64_t seq;
    const void * key;
    std::function<void()> run;
  };

  struct Ready
  {
    std::weak_ptr<rclcpp::SubscriptionBase> subscription;
    std::weak_ptr<rclcpp::ServiceBase> service;
    std::weak_ptr<rclcpp::ClientBase> client;
    const void * key;
  };

  void collect(std::vector<Ready> & ready);
  void take(const Ready & ready);
  void take_message(const Ready & ready);
  bool resumed(const void * key);
  void resume();
  void finish_spin();
  void push(const void * key, size_t limit, std::function<void()> run);

  std::thread thread_;
  std::mutex dispatch_mutex_;
  std::condition_variable resumed_;
  bool paused_ = false;
  bool waiting_ = false;
  const void * taking_ = nullptr;
  std::deque<Item> queue_;
  std::unordered_map<const void *, size_t> pending_;
  uint64_t next_seq_ = 0;
};

}  // namespace ros_integration
}  // namespace rviz_common

#endif  // RVIZ_COMMON__ROS_INTEGRATION__GUI_DISPATCH_EXECUTOR_HPP_
