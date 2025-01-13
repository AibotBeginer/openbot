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

#include "openbot/system/navigation/behavior_tree/plugins/dummy_nodes.hpp"

#include "openbot/common/utils/logging.hpp"

namespace openbot {
namespace system {
namespace navigation {
namespace behavior_tree {

// This function must be implemented in the .cpp file to create
// a plugin that can be loaded at run-time
BT_REGISTER_NODES(factory)
{
    RegisterNodes(factory);
}

BT::NodeStatus CheckBattery()
{
  std::cout << "[ Battery: OK ]" << std::endl;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CheckTemperature()
{
  std::cout << "[ Temperature: OK ]" << std::endl;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SayHello()
{
  std::cout << "Robot says: Hello World" << std::endl;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GripperInterface::open()
{
  _opened = true;
  std::cout << "GripperInterface::open" << std::endl;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GripperInterface::close()
{
  std::cout << "GripperInterface::close" << std::endl;
  _opened = false;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ApproachObject::tick()
{
  std::cout << "ApproachObject: " << this->name() << std::endl;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SaySomething::tick()
{
  auto msg = getInput<std::string>("message");
  if(!msg)
  {
    throw BT::RuntimeError("missing required input [message]: ", msg.error());
  }

  std::cout << "Robot says: " << msg.value() << std::endl;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SaySomethingSimple(BT::TreeNode& self)
{
  auto msg = self.getInput<std::string>("message");
  if(!msg)
  {
    throw BT::RuntimeError("missing required input [message]: ", msg.error());
  }

  std::cout << "Robot says: " << msg.value() << std::endl;
  return BT::NodeStatus::SUCCESS;
}

}   // namespace behavior_tree 
}   // namespace navigation
}   // namespace system
}   // namespace openbot
