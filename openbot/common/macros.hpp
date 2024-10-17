// Copyright 2024 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <utility>

#ifndef OPENBOT_COMMON__MACROS_HPP_
#define OPENBOT_COMMON__MACROS_HPP_

/**
 * Disables the copy constructor and operator= for the given class.
 *
 * Use in the private section of the class.
 */
#define OPENBOT_DISABLE_COPY(...) \
  __VA_ARGS__(const __VA_ARGS__ &) = delete; \
  __VA_ARGS__ & operator=(const __VA_ARGS__ &) = delete;

/**
 * Defines aliases and static functions for using the Class with smart pointers.
 *
 * Use in the public section of the class.
 * Make sure to include `<memory>` in the header when using this.
 */
#define OPENBOT_SMART_PTR_DEFINITIONS(...) \
  OPENBOT_SHARED_PTR_DEFINITIONS(__VA_ARGS__) \
  OPENBOT_WEAK_PTR_DEFINITIONS(__VA_ARGS__) \
  OPENBOT_UNIQUE_PTR_DEFINITIONS(__VA_ARGS__)

/**
 * Defines aliases and static functions for using the Class with smart pointers.
 *
 * Same as OPENBOT_SMART_PTR_DEFINITIONS except it excludes the static
 * Class::make_unique() method definition which does not work on classes which
 * are not CopyConstructable.
 *
 * Use in the public section of the class.
 * Make sure to include `<memory>` in the header when using this.
 */
#define OPENBOT_SMART_PTR_DEFINITIONS_NOT_COPYABLE(...) \
  OPENBOT_SHARED_PTR_DEFINITIONS(__VA_ARGS__) \
  OPENBOT_WEAK_PTR_DEFINITIONS(__VA_ARGS__) \
  __OPENBOT_UNIQUE_PTR_ALIAS(__VA_ARGS__)

/**
 * Defines aliases only for using the Class with smart pointers.
 *
 * Same as OPENBOT_SMART_PTR_DEFINITIONS except it excludes the static
 * method definitions which do not work on pure virtual classes and classes
 * which are not CopyConstructable.
 *
 * Use in the public section of the class.
 * Make sure to include `<memory>` in the header when using this.
 */
#define OPENBOT_SMART_PTR_ALIASES_ONLY(...) \
  __OPENBOT_SHARED_PTR_ALIAS(__VA_ARGS__) \
  __OPENBOT_WEAK_PTR_ALIAS(__VA_ARGS__) \
  __OPENBOT_UNIQUE_PTR_ALIAS(__VA_ARGS__) \
  __OPENBOT_MAKE_SHARED_DEFINITION(__VA_ARGS__)

#define __OPENBOT_SHARED_PTR_ALIAS(...) \
  using SharedPtr = std::shared_ptr<__VA_ARGS__>; \
  using ConstSharedPtr = std::shared_ptr<const __VA_ARGS__>;

#define __OPENBOT_MAKE_SHARED_DEFINITION(...) \
  template<typename ... Args> \
  static std::shared_ptr<__VA_ARGS__> \
  make_shared(Args && ... args) \
  { \
    return std::make_shared<__VA_ARGS__>(std::forward<Args>(args) ...); \
  }

/// Defines aliases and static functions for using the Class with shared_ptrs.
#define OPENBOT_SHARED_PTR_DEFINITIONS(...) \
  __OPENBOT_SHARED_PTR_ALIAS(__VA_ARGS__) \
  __OPENBOT_MAKE_SHARED_DEFINITION(__VA_ARGS__)

#define __OPENBOT_WEAK_PTR_ALIAS(...) \
  using WeakPtr = std::weak_ptr<__VA_ARGS__>; \
  using ConstWeakPtr = std::weak_ptr<const __VA_ARGS__>;

/// Defines aliases and static functions for using the Class with weak_ptrs.
#define OPENBOT_WEAK_PTR_DEFINITIONS(...) __OPENBOT_WEAK_PTR_ALIAS(__VA_ARGS__)

#define __OPENBOT_UNIQUE_PTR_ALIAS(...) using UniquePtr = std::unique_ptr<__VA_ARGS__>;

#define __OPENBOT_MAKE_UNIQUE_DEFINITION(...) \
  template<typename ... Args> \
  static std::unique_ptr<__VA_ARGS__> \
  make_unique(Args && ... args) \
  { \
    return std::unique_ptr<__VA_ARGS__>(new __VA_ARGS__(std::forward<Args>(args) ...)); \
  }

/// Defines aliases and static functions for using the Class with unique_ptrs.
#define OPENBOT_UNIQUE_PTR_DEFINITIONS(...) \
  __OPENBOT_UNIQUE_PTR_ALIAS(__VA_ARGS__) \
  __OPENBOT_MAKE_UNIQUE_DEFINITION(__VA_ARGS__)

#define OPENBOT_STRING_JOIN(arg1, arg2) OPENBOT_DO_STRING_JOIN(arg1, arg2)
#define OPENBOT_DO_STRING_JOIN(arg1, arg2) arg1 ## arg2

#endif  // OPENBOT_COMMON
