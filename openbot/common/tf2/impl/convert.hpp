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

#ifndef OPENBOT_COMMON_TTF2_IMPL_CONVERT_HPP_
#define OPENBOT_COMMON_TTF2_IMPL_CONVERT_HPP_

namespace openbot {
namespace common {
namespace tf2 {

namespace impl {

template <bool IS_MESSAGE_A, bool IS_MESSAGE_B>
class Converter {
public:
  template<typename A, typename B>
  static void convert(const A& a, B& b);
};

// The case where both A and B are messages should not happen: if you have two
// messages that are interchangeable, well, that's against the ROS purpose:
// only use one type. Worst comes to worst, specialize the original convert
// function for your types.
// if B == A, the templated version of convert with only one argument will be
// used.
//
//
//template <>
//template <typename A, typename B>
//inline void Converter<true, true>::convert(const A& a, B& b);

template <>
template <typename A, typename B>
inline void Converter<true, false>::convert(const A& a, B& b)
{
  fromMsg(a, b);
}

template <>
template <typename A, typename B>
inline void Converter<false, true>::convert(const A& a, B& b)
{
  b = toMsg(a);
}

template <>
template <typename A, typename B>
inline void Converter<false, false>::convert(const A& a, B& b)
{
  fromMsg(toMsg(a), b);
}

}

}  // namespace tf2
}  // namespace common
}  // namespace openbot

#endif // OPENBOT_COMMON_TTF2_IMPL_CONVERT_HPP_
