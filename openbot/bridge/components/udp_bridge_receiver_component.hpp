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
#include "openbot_bridge/chassis_msgs/chassis.pb.h"

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "cyber/init.h"
#include "cyber/scheduler/scheduler_factory.h"

#include "openbot/bridge/common/bridge_gflags.hpp"
#include "openbot/bridge/common/bridge_header.hpp"
#include "openbot/bridge/common/bridge_proto_diserialized_buf.hpp"
#include "openbot/bridge/common/udp_listener.hpp"
#include "openbot/common/monitor_log/monitor_log_buffer.hpp"

namespace openbot {
namespace bridge {
namespace components {

#define RECEIVER_BRIDGE_COMPONENT_REGISTER(pb_msg) \
  CYBER_REGISTER_COMPONENT(UDPBridgeReceiverComponent<pb_msg>)

template <typename T>
class UDPBridgeReceiverComponent final : public ::apollo::cyber::Component<> 
{
public:
  UDPBridgeReceiverComponent();
  ~UDPBridgeReceiverComponent();

  bool Init() override;

  std::string Name() const { return FLAGS_bridge_module_name; }
  bool MsgHandle(int fd);

private:
  bool InitSession(uint16_t port);
  void MsgDispatcher();
  bool IsProtoExist(const BridgeHeader &header);
  BridgeProtoDiserializedBuf<T> *CreateBridgeProtoBuf(const BridgeHeader &header);
  bool IsTimeout(double time_stamp);
  bool RemoveInvalidBuf(uint32_t msg_id);

private:
  common::monitor::MonitorLogBuffer monitor_logger_buffer_;
  unsigned int bind_port_ = 0;
  std::string proto_name_ = "";
  std::string topic_name_ = "";
  bool enable_timeout_ = true;
  std::shared_ptr<::apollo::cyber::Writer<T>> writer_;
  std::mutex mutex_;

  std::shared_ptr<UDPListener<UDPBridgeReceiverComponent<T>>> listener_ =  
    std::make_shared<UDPListener<UDPBridgeReceiverComponent<T>>>();

  std::vector<BridgeProtoDiserializedBuf<T> *> proto_list_;
};

RECEIVER_BRIDGE_COMPONENT_REGISTER(::openbot_bridge::chassis_msgs::Chassis)

}  // namespace components
}  // namespace bridge
}  // namespace openbot