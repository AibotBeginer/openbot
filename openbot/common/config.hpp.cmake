/*
 * Copyright 2016 The Cartographer Authors
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

#ifndef OPENBOT_COMMON_CONFIG_HPP_
#define OPENBOT_COMMON_CONFIG_HPP_

namespace openbot {
namespace common {

constexpr char kConfigurationFilesDirectory[] = "@OPENBOT_CONFIGURATION_FILES_DIRECTORY@";
constexpr char kSourceDirectory[] = "@PROJECT_SOURCE_DIR@";

}  // namespace common
}  // namespace openbot

#endif  // OPENBOT_COMMON_CONFIG_HPP_