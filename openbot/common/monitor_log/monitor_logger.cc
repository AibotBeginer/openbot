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

#include "openbot/common/monitor_log/monitor_logger.hpp"

#include <memory>

#include "absl/strings/str_cat.h"

using ::apollo::cyber::Time;

namespace openbot {
namespace common {
namespace monitor {

MonitorLogger::MonitorLogger() 
{
  const std::string node_name = absl::StrCat("monitor_logger", Time::Now().ToNanosecond());
  node_ = ::apollo::cyber::CreateNode(node_name);
  if (node_ != nullptr) {
    monitor_msg_writer_ = node_->CreateWriter<::openbot_bridge::monitor_msgs::MonitorMessage>("/openbot/monitor");
  }
}

void MonitorLogger::Publish(const ::openbot_bridge::monitor_msgs::MonitorMessageItem::MessageSource &source,
                            const std::vector<MessageItem> &messages) const 
{
  // compose a monitor message
  if (messages.empty()) {
    return;
  }
  ::openbot_bridge::monitor_msgs::MonitorMessage monitor_msg;

  for (const auto &msg_item : messages) {
    ::openbot_bridge::monitor_msgs::MonitorMessageItem *monitor_msg_item = monitor_msg.add_item();
    monitor_msg_item->set_source(source);
    monitor_msg_item->set_log_level(msg_item.first);
    monitor_msg_item->set_msg(msg_item.second);
  }

  // publish monitor messages
  DoPublish(&monitor_msg);
}

void MonitorLogger::DoPublish(::openbot_bridge::monitor_msgs::MonitorMessage *message) const 
{
  RETURN_IF_NULL(monitor_msg_writer_);
  common::util::FillHeader("monitor", message);
  monitor_msg_writer_->Write(*message);
}

}  // namespace monitor
}  // namespace common
}  // namespace openbot
