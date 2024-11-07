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

/**
 * @file
 * @brief Some string util functions.
 */

#pragma once

#include <string>

#include "absl/strings/str_format.h"
#include "openbot/common/utils/future.hpp"

#define FORMAT_TIMESTAMP(timestamp) \
  std::fixed << std::setprecision(9) << timestamp

/**
 * @namespace openbot::common::util
 * @brief openbot::common::util
 */
namespace openbot {
namespace common {
namespace util {

using absl::StrFormat;

struct DebugStringFormatter {
  template <class T>
  void operator()(std::string* out, const T& t) const {
    out->append(t.DebugString());
  }
};

std::string EncodeBase64(std::string_view in);

}  // namespace util
}  // namespace common
}  // namespace openbot
