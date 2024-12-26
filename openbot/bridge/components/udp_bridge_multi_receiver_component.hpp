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

class UDPBridgeMultiReceiverComponent final : public ::apollo::cyber::Component<> 
{
 public:
  UDPBridgeMultiReceiverComponent();
  ~UDPBridgeMultiReceiverComponent() = default;

  bool Init() override;
  std::string Name() const { return FLAGS_bridge_module_name; }
  std::shared_ptr<ProtoDiserializedBufBase> CreateBridgeProtoBuf(
      const BridgeHeader &header);
  bool IsProtoExist(const BridgeHeader &header);
  bool IsTimeout(double time_stamp);
  void MsgDispatcher();
  bool InitSession(uint16_t port);
  bool MsgHandle(int fd);

 private:
  bool RemoveInvalidBuf(uint32_t msg_id, const std::string &msg_name);

 private:
  common::monitor::MonitorLogBuffer monitor_logger_buffer_;
  std::shared_ptr<UDPListener<UDPBridgeMultiReceiverComponent>> listener_ =
      std::make_shared<UDPListener<UDPBridgeMultiReceiverComponent>>();
  unsigned int bind_port_ = 0;
  bool enable_timeout_ = true;
  std::mutex mutex_;
  std::vector<std::shared_ptr<ProtoDiserializedBufBase>> proto_list_;
};

CYBER_REGISTER_COMPONENT(UDPBridgeMultiReceiverComponent)

}  // namespace bridge
}  // namespace openbot
