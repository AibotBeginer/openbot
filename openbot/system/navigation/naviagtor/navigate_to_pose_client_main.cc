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

#include <vector>

#include "gtest/gtest.h"

#include "cyber/cyber.h"

#include "openbot/system/navigation/proto/navigate_to_pose.pb.h"

int main(int argc, char* argv[]) 
{
    apollo::cyber::Init(argv[0]);
    std::shared_ptr<apollo::cyber::Node> node(apollo::cyber::CreateNode("navigator"));

    auto client = node->CreateClient<
      openbot::navigation::NavigateToPose::Request, 
      openbot::navigation::NavigateToPose::Response>("navigate_to_pose");
    auto request_msg = std::make_shared<openbot::navigation::NavigateToPose::Request>();

    request_msg->set_command_type(openbot::navigation::CommandType::ACTIVATE);
    request_msg->set_behavior_tree("111111111111111111");

    while (apollo::cyber::OK()) {
      auto res = client->SendRequest(request_msg);
      if (res != nullptr) {
        AINFO << "client: response: " << res->ShortDebugString();
      } else {
        AINFO << "client: service may not ready.";
      }
      break;
    }

  apollo::cyber::WaitForShutdown();
  return 0;
}
