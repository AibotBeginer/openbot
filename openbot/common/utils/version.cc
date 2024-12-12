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

#include "openbot/common/utils/version.hpp"
#include "openbot/common/utils/string_util.hpp"

namespace openbot {
namespace common {
namespace utils {
namespace {

const char* OPENBOT_VERSION = "1.0.1";
const char* OPENBOT_COMMIT_ID = "a2cd16b";
const char* OPENBOT_COMMIT_DATE = "2024-12-12";

}  // namespace

std::string GetVersionInfo() {
  return StringPrintf("OPENBOT %s", OPENBOT_VERSION);
}

std::string GetBuildInfo() {
#if defined(OPENBOT_CUDA_ENABLED)
  const char* cuda_info = "with CUDA";
#else
  const char* cuda_info = "without CUDA";
#endif
  return StringPrintf(
      "Openbot develop info: Commit %s on %s %s", OPENBOT_COMMIT_ID, OPENBOT_COMMIT_DATE, cuda_info);
}

void ShowVersion() 
{
  std::cout << StringPrintf("%s -- Openbot for robotics develop everyone \n(%s)",
      GetVersionInfo().c_str(),
      GetBuildInfo().c_str()) 
      << std::endl 
      << std::endl;
}

}  // namespace utils
}  // namespace common
}  // namespace openbot 

