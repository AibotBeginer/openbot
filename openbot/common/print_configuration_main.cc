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

#include <iostream>
#include <vector>

#include "gflags/gflags.h"
#include "glog/logging.h"

namespace openbot {
namespace common {


}  // namespace common
}  // namespace openbot

int main(int argc, char** argv) 
{
  // google::InitGoogleLogging(argv[0]);
  // google::SetUsageMessage(
  //     "Resolves and compiles a Lua configuration and prints it to stdout.\n"
  //     "The output can be restricted to a subdictionary using the optional "
  //     "'--subdictionary' parameter, which can be given in Lua syntax.\n"
  //     "The logs of the configuration file resolver are written to stderr if "
  //     "'--logtostderr' is given.");
  // google::ParseCommandLineFlags(&argc, &argv, true);


  LOG(INFO) << "Start first flying !!!" ;
  return 0;
}
