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

#pragma once

#include <netinet/in.h>
#include <sys/socket.h>

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "openbot/bridge/proto/udp_bridge_remote_info.pb.h"
// #include "openbot/common_msgs/planning_msgs/planning.pb.h"

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "cyber/init.h"
#include "cyber/io/session.h"
#include "cyber/scheduler/scheduler_factory.h"

#include "openbot/bridge/common/bridge_gflags.hpp"
#include "openbot/common/monitor_log/monitor_log_buffer.hpp"
#include "openbot/common/utils/util.hpp"

namespace openbot {
namespace bridge {
namespace components {

#define BRIDGE_COMPONENT_REGISTER(pb_msg) \
  CYBER_REGISTER_COMPONENT(UDPBridgeSenderComponent<pb_msg>)

template <typename T>
class UDPBridgeSenderComponent final : public ::apollo::cyber::Component<T> 
{
public:
  UDPBridgeSenderComponent()
      : monitor_logger_buffer_(openbot_bridge::monitor_msgs::MonitorMessageItem::CONTROL) {}

  bool Init() override;
  bool Proc(const std::shared_ptr<T> &pb_msg) override;

  std::string Name() const { return FLAGS_bridge_module_name; }

private:
  common::monitor::MonitorLogBuffer monitor_logger_buffer_;
  unsigned int remote_port_ = 0;
  std::string remote_ip_ = "";
  std::string proto_name_ = "";
  std::mutex mutex_;
};

// BRIDGE_COMPONENT_REGISTER(planning::ADCTrajectory)
// BRIDGE_COMPONENT_REGISTER(localization::LocalizationEstimate)

}  // namespace components
}  // namespace bridge
}  // namespace openbot
