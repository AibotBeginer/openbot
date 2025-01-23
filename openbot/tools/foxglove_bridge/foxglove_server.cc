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


#include "openbot/tools/foxglove_bridge/foxglove_server.hpp"

namespace openbot {
namespace tools {
namespace foxglove_bridge {

FoxgloveServer::FoxgloveServer()
{

}

FoxgloveServer::~FoxgloveServer()
{

}

openbot::common::Status FoxgloveServer::Init()
{
    return openbot::common::Status::OK();
}

openbot::common::Status FoxgloveServer::Start()
{
    return openbot::common::Status::OK();
}

void FoxgloveServer::Stop()
{

}


}  // namespace foxglove_bridge
}  // namespace tools
}  // namespace openbot