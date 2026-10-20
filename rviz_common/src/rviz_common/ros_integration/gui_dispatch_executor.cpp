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

#include "gui_dispatch_executor.hpp"

#include <pthread.h>

#include <stdexcept>
#include <unordered_set>
#include <utility>

#include "rcl/error_handling.h"
#include "rcl/subscription.h"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/logging.hpp"
#include "rcpputils/scope_exit.hpp"

namespace rviz_common
{
namespace ros_integration
{

namespace
{

const rclcpp::Logger & logger()
{
  static const rclcpp::Logger log = rclcpp::get_logger("rviz_common.executor");
  return log;
}

template<typename Take>
bool take_logged(const char * what, const char * name, Take take)
{
  try {
    return take();
  } catch (const rclcpp::exceptions::RCLError & e) {
    RCLCPP_ERROR(logger(), "executor %s '%s' unexpectedly failed: %s", what, name, e.what());
    return false;
  }
}

}  // namespace

GuiDispatchExecutor::GuiDispatchExecutor(const rclcpp::ExecutorOptions & options)
: rclcpp::Executor(options)
{}

GuiDispatchExecutor::~GuiDispatchExecutor()
{
  stop();
}

void GuiDispatchExecutor::start()
{
  thread_ = std::thread(
    [this]() {
      pthread_setname_np(pthread_self(), "rviz_ros");
      spin();
    });
}

void GuiDispatchExecutor::stop()
{
  if (thread_.joinable()) {
    cancel();
    resume();
    thread_.join();
  }
  std::deque<Item> dropped;
  std::lock_guard<std::mutex> lock(dispatch_mutex_);
  dropped.swap(queue_);
  pending_.clear();
}

size_t GuiDispatchExecutor::pending()
{
  std::lock_guard<std::mutex> lock(dispatch_mutex_);
  return queue_.size();
}

bool GuiDispatchExecutor::resumed(const void * key)
{
  std::unique_lock<std::mutex> lock(dispatch_mutex_);
  resumed_.wait(lock, [this]() {return !paused_ || !spinning.load();});
  waiting_ = key == nullptr;
  taking_ = key;
  return spinning.load();
}

void GuiDispatchExecutor::resume()
{
  {
    std::lock_guard<std::mutex> lock(dispatch_mutex_);
    paused_ = false;
  }
  resumed_.notify_all();
}

void GuiDispatchExecutor::finish_spin()
{
  {
    std::lock_guard<std::mutex> lock(dispatch_mutex_);
    waiting_ = false;
    taking_ = nullptr;
  }
  resumed_.notify_all();
  spinning.store(false);
}

void GuiDispatchExecutor::spin()
{
  if (spinning.exchange(true)) {
    throw std::runtime_error("spin() called while already spinning");
  }
  RCPPUTILS_SCOPE_EXIT(this->finish_spin(); );
  std::vector<Ready> ready;
  while (rclcpp::ok(context_) && resumed(nullptr)) {
    wait_for_work(std::chrono::nanoseconds(-1));
    collect(ready);
    wait_result_.reset();
    {
      std::lock_guard<std::mutex> lock(dispatch_mutex_);
      waiting_ = false;
    }
    resumed_.notify_all();
    for (const auto & r : ready) {
      if (!resumed(r.key)) {
        break;
      }
      take(r);
      std::lock_guard<std::mutex> lock(dispatch_mutex_);
      taking_ = nullptr;
    }
    ready.clear();
  }
}

void GuiDispatchExecutor::collect(std::vector<Ready> & ready)
{
  rclcpp::AnyExecutable any;
  while (get_next_ready_executable(any)) {
    // The callbacks run serially in dispatch(), so the group needs no exclusion here.
    any.callback_group->can_be_taken_from().store(true);
    if (any.timer) {
      std::weak_ptr<rclcpp::TimerBase> weak = any.timer;
      push(
        any.timer.get(), 1, [weak, data = std::move(any.data)]() {
          if (auto timer = weak.lock()) {
            timer->execute_callback(data);
          }
        });
    } else if (any.waitable) {
      std::weak_ptr<rclcpp::Waitable> weak = any.waitable;
      push(
        any.waitable.get(), 0, [weak, data = std::move(any.data)]() {
          if (auto waitable = weak.lock()) {
            waitable->execute(data);
          }
        });
    } else if (any.subscription) {
      ready.push_back({any.subscription, {}, {}, any.subscription.get()});
    } else if (any.service) {
      ready.push_back({{}, any.service, {}, any.service.get()});
    } else if (any.client) {
      ready.push_back({{}, {}, any.client, any.client.get()});
    }
    any = rclcpp::AnyExecutable();
  }
}

void GuiDispatchExecutor::take(const Ready & ready)
{
  // Each push happens after the strong reference is gone, so dispatch() can lock an entity
  // only while its owner still holds it.
  if (!ready.subscription.expired()) {
    take_message(ready);
  } else if (auto srv = ready.service.lock()) {
    auto header = srv->create_request_header();
    auto request = srv->create_request();
    if (take_logged(
        "taking a service server request from service", srv->get_service_name(),
        [&]() {return srv->take_type_erased_request(request.get(), *header);}))
    {
      srv.reset();
      push(
        ready.key, 0, [weak = ready.service, header = std::move(header),
        request = std::move(request)]() {
          if (auto s = weak.lock()) {
            s->handle_request(header, request);
          }
        });
    }
  } else if (auto client = ready.client.lock()) {
    auto header = client->create_request_header();
    auto response = client->create_response();
    if (take_logged(
        "taking a service client response from service", client->get_service_name(),
        [&]() {return client->take_type_erased_response(response.get(), *header);}))
    {
      client.reset();
      push(
        ready.key, 0, [weak = ready.client, header = std::move(header),
        response = std::move(response)]() {
          if (auto c = weak.lock()) {
            c->handle_response(header, response);
          }
        });
    }
  }
}

void GuiDispatchExecutor::take_message(const Ready & ready)
{
  std::shared_ptr<rcl_subscription_t> handle;
  std::shared_ptr<void> msg;
  std::shared_ptr<rclcpp::SerializedMessage> serialized;
  size_t limit;
  rclcpp::MessageInfo info;
  info.get_rmw_message_info().from_intra_process = false;
  {
    auto sub = ready.subscription.lock();
    if (!sub) {
      return;
    }
    const auto qos = sub->get_actual_qos().get_rmw_qos_profile();
    limit = qos.history == RMW_QOS_POLICY_HISTORY_KEEP_LAST ? qos.depth : 0;
    switch (sub->get_delivered_message_kind()) {
      case rclcpp::DeliveredMessageKind::ROS_MESSAGE:
        msg = sub->create_message();
        break;
      case rclcpp::DeliveredMessageKind::SERIALIZED_MESSAGE:
        serialized = sub->create_serialized_message();
        break;
      default:
        RCLCPP_ERROR(
          logger(), "delivered message kind of topic '%s' is not supported", sub->get_topic_name());
        return;
    }
    if (msg && sub->get_intra_process_waitable()) {
      // take_type_erased() drops the copies that intra-process delivery also brings.
      if (!take_logged(
          "taking a message from topic", sub->get_topic_name(),
          [&]() {return sub->take_type_erased(msg.get(), info);}))
      {
        return;
      }
    } else {
      handle = sub->get_subscription_handle();
    }
  }
  if (handle) {
    // The rcl handle keeps the reader alive without the subscription, whose owner may release it
    // while a large message deserializes.
    {
      std::lock_guard<std::mutex> lock(dispatch_mutex_);
      taking_ = nullptr;
    }
    const rcl_ret_t ret = msg ?
      rcl_take(handle.get(), msg.get(), &info.get_rmw_message_info(), nullptr) :
      rcl_take_serialized_message(
      handle.get(), &serialized->get_rcl_serialized_message(), &info.get_rmw_message_info(),
      nullptr);
    if (ret == RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
      return;
    }
    if (ret != RCL_RET_OK) {
      RCLCPP_ERROR(
        logger(), "executor taking a message from topic '%s' unexpectedly failed: %s",
        rcl_subscription_get_topic_name(handle.get()), rcl_get_error_string().str);
      rcl_reset_error();
      return;
    }
  }
  if (msg) {
    push(
      ready.key, limit, [weak = ready.subscription, msg = std::move(msg), info]() mutable {
        if (auto s = weak.lock()) {
          s->handle_message(msg, info);
          s->return_message(msg);
        }
      });
  } else {
    push(
      ready.key, limit, [weak = ready.subscription, msg = std::move(serialized), info]() mutable {
        if (auto s = weak.lock()) {
          s->handle_serialized_message(msg, info);
          s->return_serialized_message(msg);
        }
      });
  }
}

void GuiDispatchExecutor::push(const void * key, size_t limit, std::function<void()> run)
{
  std::function<void()> dropped;
  std::lock_guard<std::mutex> lock(dispatch_mutex_);
  auto & count = pending_[key];
  if (limit > 0 && count >= limit) {
    for (auto it = queue_.begin(); it != queue_.end(); ++it) {
      if (it->key == key) {
        dropped = std::move(it->run);
        queue_.erase(it);
        --count;
        break;
      }
    }
  }
  queue_.push_back(Item{next_seq_++, key, std::move(run)});
  ++count;
  if (taking_ == key) {
    taking_ = nullptr;
  }
}

void GuiDispatchExecutor::dispatch(std::chrono::nanoseconds max_duration)
{
  if (!rclcpp::ok(context_)) {
    return;
  }
  const auto start = std::chrono::steady_clock::now();
  std::unordered_set<const void *> served;
  uint64_t end_seq;
  {
    std::unique_lock<std::mutex> lock(dispatch_mutex_);
    paused_ = true;
    if (waiting_) {
      interrupt_guard_condition_->trigger();
      resumed_.wait(lock, [this]() {return !waiting_;});
    }
    // The entity being taken is still held by the ROS thread; its callbacks wait a call.
    if (taking_) {
      served.insert(taking_);
    }
    end_seq = next_seq_;
  }
  RCPPUTILS_SCOPE_EXIT(this->resume(); );
  while (rclcpp::ok(context_)) {
    std::function<void()> run;
    {
      std::lock_guard<std::mutex> lock(dispatch_mutex_);
      auto it = queue_.begin();
      while (it != queue_.end() && it->seq < end_seq && served.count(it->key)) {
        ++it;
      }
      if (it == queue_.end() || it->seq >= end_seq) {
        break;
      }
      served.insert(it->key);
      run = std::move(it->run);
      auto count = pending_.find(it->key);
      if (--count->second == 0) {
        pending_.erase(count);
      }
      queue_.erase(it);
    }
    run();
    if (max_duration.count() > 0 && std::chrono::steady_clock::now() - start >= max_duration) {
      break;
    }
  }
}

}  // namespace ros_integration
}  // namespace rviz_common
