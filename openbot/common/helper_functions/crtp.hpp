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

#ifndef OPENBOT_COMMON_HELPER_FUNCTIONS__CRTP_HPP_
#define OPENBOT_COMMON_HELPER_FUNCTIONS__CRTP_HPP_

namespace openbot {
namespace common {
namespace helper_functions {

template <typename Derived>
class crtp
{
protected:
  const Derived & impl() const
  {
    // This is the CRTP pattern for static polymorphism: this is related, static_cast is the only
    // way to do this
    // lint -e{9005, 9176, 1939} NOLINT
    return *static_cast<const Derived *>(this);
  }

  Derived & impl()
  {
    // This is the CRTP pattern for static polymorphism: this is related, static_cast is the only
    // way to do this
    // lint -e{9005, 9176, 1939} NOLINT
    return *static_cast<Derived *>(this);
  }
};

}  // namespace helper_functions
}  // namespace common
}  // namespace openbot

#endif  // OPENBOT_COMMON_HELPER_FUNCTIONS__CRTP_HPP_
