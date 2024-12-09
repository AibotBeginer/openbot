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


#include "gflags/gflags.h"
#include "glog/logging.h"

#include "openbot/common/proto/nav_msgs/path.pb.h"

#include "cyber/cyber.h"
#include "cyber/time/rate.h"
#include "cyber/time/time.h"

using apollo::cyber::Rate;
using apollo::cyber::Time;

namespace openbot {
namespace {

void Run() 
{
}

}  // namespace
}  // namespace openbot

int main(int argc, char *argv[]) {
  // init cyber framework
  apollo::cyber::Init(argv[0]);

  // create talker node
  auto talker_node = apollo::cyber::CreateNode("talker");

  // create talker
  auto talker = talker_node->CreateWriter<openbot::common::proto::nav_msgs::Path>("path");
  Rate rate(1.0);
  uint64_t seq = 0;
  while (apollo::cyber::OK()) {
    auto msg = std::make_shared<openbot::common::proto::nav_msgs::Path>();
    talker->Write(msg);
    LOG(INFO) << "talker sent a message! No. " << seq;
    seq++;
    rate.Sleep();
  }
  return 0;
}