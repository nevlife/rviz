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

#include <gmock/gmock.h>

#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#include "../src/rviz_common/ros_integration/gui_dispatch_executor.hpp"

using namespace ::testing;  // NOLINT
using rviz_common::ros_integration::GuiDispatchExecutor;
using std_msgs::msg::Int32;

class GuiDispatchExecutorTest : public Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>(
      "gui_dispatch_executor_test",
      rclcpp::NodeOptions().start_parameter_event_publisher(false).start_parameter_services(false));
    executor_ = std::make_shared<GuiDispatchExecutor>();
    executor_->add_node(node_);
  }

  void TearDown() override
  {
    executor_->stop();
    executor_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  rclcpp::Subscription<Int32>::SharedPtr subscribe(size_t depth)
  {
    return node_->create_subscription<Int32>(
      "numbers", rclcpp::QoS(depth).reliable(), [this](Int32::ConstSharedPtr msg) {
        values_.push_back(msg->data);
        threads_.push_back(std::this_thread::get_id());
      });
  }

  void publish(int count)
  {
    auto pub = node_->create_publisher<Int32>("numbers", rclcpp::QoS(10).reliable());
    ASSERT_TRUE(waitFor([&]() {return pub->get_subscription_count() > 0;}));
    for (int i = 0; i < count; ++i) {
      Int32 msg;
      msg.data = i;
      pub->publish(msg);
    }
  }

  void dispatchAll()
  {
    for (int i = 0; i < 100 && executor_->pending() > 0; ++i) {
      executor_->dispatch(std::chrono::milliseconds(10));
    }
    EXPECT_THAT(executor_->pending(), Eq(0u));
  }

  static bool waitFor(const std::function<bool()> & done)
  {
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (!done()) {
      if (std::chrono::steady_clock::now() > end) {
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return true;
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<GuiDispatchExecutor> executor_;
  std::vector<int> values_;
  std::vector<std::thread::id> threads_;
};

TEST_F(GuiDispatchExecutorTest, takes_on_its_thread_and_runs_callbacks_in_dispatch) {
  auto sub = subscribe(10);
  executor_->start();
  publish(3);
  ASSERT_TRUE(waitFor([&]() {return executor_->pending() == 3;}));
  EXPECT_THAT(values_, IsEmpty());

  // A dispatch runs a subscription at most once; it defers one whose message is being taken.
  executor_->dispatch(std::chrono::milliseconds(10));
  EXPECT_THAT(values_.size(), Le(1u));
  dispatchAll();
  EXPECT_THAT(values_, ElementsAre(0, 1, 2));
  EXPECT_THAT(threads_, Each(Eq(std::this_thread::get_id())));
}

TEST_F(GuiDispatchExecutorTest, keeps_the_newest_messages_up_to_the_history_depth) {
  auto sub = subscribe(2);
  executor_->start();
  publish(5);
  ASSERT_TRUE(waitFor([&]() {return executor_->pending() == 2;}));
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  dispatchAll();
  EXPECT_THAT(values_, ElementsAre(3, 4));
}

TEST_F(GuiDispatchExecutorTest, drops_messages_of_a_destroyed_subscription) {
  auto sub = subscribe(10);
  executor_->start();
  publish(1);
  ASSERT_TRUE(waitFor([&]() {return executor_->pending() == 1;}));
  sub.reset();
  dispatchAll();
  EXPECT_THAT(values_, IsEmpty());
}

TEST_F(GuiDispatchExecutorTest, runs_a_timer_once_per_dispatch_on_the_dispatching_thread) {
  int calls = 0;
  std::thread::id thread;
  auto timer = node_->create_wall_timer(
    std::chrono::milliseconds(5), [&]() {
      ++calls;
      thread = std::this_thread::get_id();
    });
  executor_->start();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_THAT(calls, Eq(0));
  EXPECT_THAT(executor_->pending(), Eq(1u));
  executor_->dispatch(std::chrono::milliseconds(10));
  EXPECT_THAT(calls, Eq(1));
  EXPECT_THAT(thread, Eq(std::this_thread::get_id()));
}

TEST_F(GuiDispatchExecutorTest, stop_discards_pending_work) {
  auto sub = subscribe(10);
  executor_->start();
  publish(2);
  ASSERT_TRUE(waitFor([&]() {return executor_->pending() == 2;}));
  executor_->stop();
  EXPECT_THAT(executor_->pending(), Eq(0u));
  executor_->dispatch(std::chrono::milliseconds(10));
  EXPECT_THAT(values_, IsEmpty());
}

TEST_F(GuiDispatchExecutorTest, shutdown_ends_the_ros_thread) {
  auto sub = subscribe(10);
  executor_->start();
  publish(1);
  ASSERT_TRUE(waitFor([&]() {return executor_->pending() == 1;}));
  rclcpp::shutdown();
  executor_->dispatch(std::chrono::milliseconds(10));
  EXPECT_THAT(values_, IsEmpty());
  executor_->stop();
}
