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


#ifndef OPENBOT_COMMON_TF2_ERROR_HPP_
#define OPENBOT_COMMON_TF2_ERROR_HPP_

namespace openbot {
namespace common {
namespace tf2_msgs {
namespace TF2Error {

const uint8_t NO_ERROR = 0;
const uint8_t LOOKUP_ERROR = 1;
const uint8_t CONNECTIVITY_ERROR = 2;
const uint8_t EXTRAPOLATION_ERROR = 3;
const uint8_t INVALID_ARGUMENT_ERROR = 4;
const uint8_t TIMEOUT_ERROR = 5;
const uint8_t TRANSFORM_ERROR = 6;

}  // namespace TF2Error
}  // namespace tf2
}  // namespace common
}  // namespace openbot

#endif  // OPENBOT_COMMON_TF2_ERROR_HPP_
