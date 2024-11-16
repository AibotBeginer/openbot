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


#pragma once

#include "openbot/common/utils/string_util.hpp"

#include <exception>
#include <iostream>

#include <glog/logging.h>

// Option checker macros. In contrast to glog, this function does not abort the
// program, but simply returns false on failure.
#define CHECK_OPTION_IMPL(expr) \
  openbot::common::utils::__CheckOptionImpl(__FILE__, __LINE__, (expr), #expr)
#define CHECK_OPTION(expr)                                             \
  if (!openbot::__CheckOptionImpl(__FILE__, __LINE__, (expr), #expr)) { \
    return false;                                                      \
  }
#define CHECK_OPTION_OP(name, op, val1, val2)      \
  if (!openbot::common::utils::__CheckOptionOpImpl(__FILE__,       \
                                   __LINE__,       \
                                   (val1 op val2), \
                                   val1,           \
                                   val2,           \
                                   #val1,          \
                                   #val2,          \
                                   #op)) {         \
    return false;                                  \
  }
#define CHECK_OPTION_EQ(val1, val2) CHECK_OPTION_OP(_EQ, ==, val1, val2)
#define CHECK_OPTION_NE(val1, val2) CHECK_OPTION_OP(_NE, !=, val1, val2)
#define CHECK_OPTION_LE(val1, val2) CHECK_OPTION_OP(_LE, <=, val1, val2)
#define CHECK_OPTION_LT(val1, val2) CHECK_OPTION_OP(_LT, <, val1, val2)
#define CHECK_OPTION_GE(val1, val2) CHECK_OPTION_OP(_GE, >=, val1, val2)
#define CHECK_OPTION_GT(val1, val2) CHECK_OPTION_OP(_GT, >, val1, val2)

// Alternative checks to throw an exception instead of aborting the program.
// Usage: THROW_CHECK(condition) << message;
//        THROW_CHECK_EQ(val1, val2) << message;
//        LOG(FATAL_THROW) << message;
// These macros are copied from glog/logging.h and extended to a new severity
// level FATAL_THROW.
#define COMPACT_GOOGLE_LOG_FATAL_THROW \
  openbot::common::utils::LogMessageFatalThrowDefault(__FILE__, __LINE__)

#define LOG_TO_STRING_FATAL_THROW(message) \
  openbot::common::utils::LogMessageFatalThrowDefault(__FILE__, __LINE__, message)

#define LOG_FATAL_THROW(exception) \
  openbot::common::utils::LogMessageFatalThrow<exception>(__FILE__, __LINE__).stream()

#define THROW_CHECK(condition)                                       \
  LOG_IF(FATAL_THROW, GOOGLE_PREDICT_BRANCH_NOT_TAKEN(!(condition))) \
      << "Check failed: " #condition " "

#define THROW_CHECK_OP(name, op, val1, val2) \
  CHECK_OP_LOG(name, op, val1, val2, openbot::common::utils::LogMessageFatalThrowDefault)

#define THROW_CHECK_EQ(val1, val2) THROW_CHECK_OP(_EQ, ==, val1, val2)
#define THROW_CHECK_NE(val1, val2) THROW_CHECK_OP(_NE, !=, val1, val2)
#define THROW_CHECK_LE(val1, val2) THROW_CHECK_OP(_LE, <=, val1, val2)
#define THROW_CHECK_LT(val1, val2) THROW_CHECK_OP(_LT, <, val1, val2)
#define THROW_CHECK_GE(val1, val2) THROW_CHECK_OP(_GE, >=, val1, val2)
#define THROW_CHECK_GT(val1, val2) THROW_CHECK_OP(_GT, >, val1, val2)

#define THROW_CHECK_NOTNULL(val) \
  openbot::common::utils::ThrowCheckNotNull(     \
      __FILE__, __LINE__, "'" #val "' Must be non NULL", (val))

namespace openbot {
namespace common {
namespace utils {

// Initialize glog at the beginning of the program.
void InitializeGlog(char** argv);

////////////////////////////////////////////////////////////////////////////////
// Implementation
////////////////////////////////////////////////////////////////////////////////

const char* __GetConstFileBaseName(const char* file);

bool __CheckOptionImpl(const char* file,
                       int line,
                       bool result,
                       const char* expr_str);

template <typename T1, typename T2>
bool __CheckOptionOpImpl(const char* file,
                         const int line,
                         const bool result,
                         const T1& val1,
                         const T2& val2,
                         const char* val1_str,
                         const char* val2_str,
                         const char* op_str) {
  if (result) {
    return true;
  } else {
    LOG(ERROR) << StringPrintf("[%s:%d] Check failed: %s %s %s (%s vs. %s)",
                               __GetConstFileBaseName(file),
                               line,
                               val1_str,
                               op_str,
                               val2_str,
                               std::to_string(val1).c_str(),
                               std::to_string(val2).c_str());
    return false;
  }
}

inline std::string __MakeExceptionPrefix(const char* file, int line) {
  return "[" + std::string(__GetConstFileBaseName(file)) + ":" +
         std::to_string(line) + "] ";
}

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4722)
#endif

template <typename T>
class LogMessageFatalThrow : public google::LogMessage {
 public:
  LogMessageFatalThrow(const char* file, int line)
      : google::LogMessage(file, line, google::GLOG_ERROR, &message_),
        prefix_(__MakeExceptionPrefix(file, line)) {}
  LogMessageFatalThrow(const char* file, int line, std::string* message)
      : google::LogMessage(file, line, google::GLOG_ERROR, message),
        message_(*message),
        prefix_(__MakeExceptionPrefix(file, line)) {}
  LogMessageFatalThrow(const char* file,
                       int line,
#if defined(GLOG_VERSION_MAJOR) && \
    (GLOG_VERSION_MAJOR > 0 || GLOG_VERSION_MINOR >= 7)
                       const google::logging::internal::CheckOpString& result)
#else
                       const google::CheckOpString& result)
#endif
      : google::LogMessage(file, line, google::GLOG_ERROR, &message_),
        prefix_(__MakeExceptionPrefix(file, line)) {
    stream() << "Check failed: " << (*result.str_) << " ";
    // On LOG(FATAL) glog<0.7.0 does not bother cleaning up CheckOpString
    // so we do it here.
#if !(defined(GLOG_VERSION_MAJOR) && \
      (GLOG_VERSION_MAJOR > 0 || GLOG_VERSION_MINOR >= 7))
    delete result.str_;
#endif
  }
  ~LogMessageFatalThrow() noexcept(false) {
    Flush();
#if defined(__cpp_lib_uncaught_exceptions) && \
    (__cpp_lib_uncaught_exceptions >= 201411L)
    if (std::uncaught_exceptions() == 0)
#else
    if (!std::uncaught_exception())
#endif
    {
      throw T(prefix_ + message_);
    }
  }

 private:
  std::string message_;
  std::string prefix_;
};

#ifdef _MSC_VER
#pragma warning(pop)
#endif

using LogMessageFatalThrowDefault = LogMessageFatalThrow<std::invalid_argument>;

template <typename T>
T ThrowCheckNotNull(const char* file, int line, const char* names, T&& t) {
  if (GOOGLE_PREDICT_FALSE(t == nullptr)) {
    LogMessageFatalThrowDefault(file, line).stream() << names;
  }
  return std::forward<T>(t);
}

}  // namespace utils
}  // namespace common
}  // namespace openbot

