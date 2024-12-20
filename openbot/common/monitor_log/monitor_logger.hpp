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


/**
 * @file monitor_logger.h
 * @brief The class of MonitorLogger
 */

#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "cyber/cyber.h"
#include "openbot/common/utils/message_util.hpp"
#include "openbot_bridge/monitor_msgs/monitor_log.pb.h"

/**
 * @namespace openbot::common::monitor
 * @brief openbot::common::monitor
 */
namespace openbot {
namespace common {
namespace monitor {

using MessageItem = std::pair<::openbot_bridge::monitor_msgs::MonitorMessageItem::LogLevel, std::string>;

/**
 * class MonitorLogger
 *
 * @brief This class helps collect and publish MonitorMessage pb to monitor
 * topic. A module who wants to publish message can use macro
 * `MONITOR(log_level, log_msg)` to record messages, and call
 * Publish to broadcast the message to other modules.
 */
class MonitorLogger 
{
 public:
  virtual ~MonitorLogger() = default;

  /**
   * @brief Publish the messages.
   * @param messages a list of messages for
   */
  virtual void Publish(const ::openbot_bridge::monitor_msgs::MonitorMessageItem::MessageSource &source,
                       const std::vector<MessageItem> &messages) const;

private:
  virtual void DoPublish(::openbot_bridge::monitor_msgs::MonitorMessage *message) const;

  ::openbot_bridge::monitor_msgs::MonitorMessageItem::MessageSource source_;
  std::unique_ptr<::apollo::cyber::Node> node_;
  std::shared_ptr<::apollo::cyber::Writer<::openbot_bridge::monitor_msgs::MonitorMessage>> monitor_msg_writer_;

  DECLARE_SINGLETON(MonitorLogger)
};

}  // namespace monitor
}  // namespace common
}  // namespace openbot
