/*
 * Copyright 2024 The OpenRobotic Beginner Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include <sys/resource.h>
#include <sys/time.h>

#include "openbot/common/monitor_log/monitor_log_buffer.hpp"

#include "gtest/gtest.h"

#include "cyber/common/log.h"
#include "cyber/init.h"

namespace openbot {
namespace common {
namespace monitor {

class MonitorBufferTest : public ::testing::Test 
{
 protected:
  void SetUp() override { apollo::cyber::Init("monitor_log_buffer_test"); }
  void TearDown() override {}
  MonitorLogBuffer buffer_{::openbot_bridge::monitor_msgs::MonitorMessageItem::CONTROL};
};

TEST_F(MonitorBufferTest, RegisterMacro) 
{
  {
    buffer_.INFO("Info");
    EXPECT_EQ(::openbot_bridge::monitor_msgs::MonitorMessageItem::INFO, buffer_.level_);
    ASSERT_EQ(0, buffer_.monitor_msg_items_.size());
  }

  {
    buffer_.ERROR("Error");
    EXPECT_EQ(::openbot_bridge::monitor_msgs::MonitorMessageItem::INFO, buffer_.level_);
    ASSERT_EQ(0, buffer_.monitor_msg_items_.size());
  }
}

TEST_F(MonitorBufferTest, AddMonitorMsgItem) 
{
  buffer_.AddMonitorMsgItem(::openbot_bridge::monitor_msgs::MonitorMessageItem::ERROR, "TestError");
  EXPECT_EQ(::openbot_bridge::monitor_msgs::MonitorMessageItem::ERROR, buffer_.level_);
  ASSERT_EQ(1, buffer_.monitor_msg_items_.size());
  const auto &item = buffer_.monitor_msg_items_.back();
  EXPECT_EQ(::openbot_bridge::monitor_msgs::MonitorMessageItem::ERROR, item.first);
  EXPECT_EQ("TestError", item.second);
}

}  // namespace monitor
}  // namespace common
}  // namespace openbot
