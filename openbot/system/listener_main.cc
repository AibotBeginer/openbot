/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "openbot/common/proto/nav_msgs/path.pb.h"

#include "cyber/cyber.h"

void MessageCallback(const std::shared_ptr<openbot::common::proto::nav_msgs::Path>& msg) 
{
  std::cout << "msgcontent->" << std::endl;
}

int main(int argc, char* argv[]) {
  // init cyber framework
  apollo::cyber::Init(argv[0]);
  // create listener node
  auto listener_node = apollo::cyber::CreateNode("path");
  // create listener
  auto listener =
      listener_node->CreateReader<openbot::common::proto::nav_msgs::Path>("path", MessageCallback);
  apollo::cyber::WaitForShutdown();
  return 0;
}