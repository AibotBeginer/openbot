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


#ifndef OPENBOT_COMMON_TF2_EXCEPTIONS_HPP_
#define OPENBOT_COMMON_TF2_EXCEPTIONS_HPP_

#include <stdexcept>

namespace openbot {
namespace common {
namespace tf2 {


/** \brief A base class for all tf2 exceptions 
 * This inherits from ros::exception 
 * which inherits from std::runtime_exception
 */
class TransformException: public std::runtime_error
{ 
public:
  TransformException(const std::string errorDescription) : std::runtime_error(errorDescription) { ; };
};

  /** \brief An exception class to notify of no connection
   * 
   * This is an exception class to be thrown in the case 
   * that the Reference Frame tree is not connected between
   * the frames requested. */
class ConnectivityException:public TransformException
{ 
public:
  ConnectivityException(const std::string errorDescription) : tf2::TransformException(errorDescription) { ; };
};


/** \brief An exception class to notify of bad frame number 
 * 
 * This is an exception class to be thrown in the case that 
 * a frame not in the graph has been attempted to be accessed.
 * The most common reason for this is that the frame is not
 * being published, or a parent frame was not set correctly 
 * causing the tree to be broken.  
 */
class LookupException: public TransformException
{ 
public:
  LookupException(const std::string errorDescription) : tf2::TransformException(errorDescription) { ; };
};

  /** \brief An exception class to notify that the requested value would have required extrapolation beyond current limits.
   * 
   */
class ExtrapolationException: public TransformException 
{ 
public:
  ExtrapolationException(const std::string errorDescription) : tf2::TransformException(errorDescription) { ; };
};

/** \brief An exception class to notify that one of the arguments is invalid
 * 
 * usually it's an uninitalized Quaternion (0,0,0,0)
 * 
 */
class InvalidArgumentException: public TransformException  
{ 
public:
  InvalidArgumentException(const std::string errorDescription) : tf2::TransformException(errorDescription) { ; };
};

/** \brief An exception class to notify that a timeout has occured
 * 
 * 
 */
class TimeoutException: public TransformException  
{ 
public:
  TimeoutException(const std::string errorDescription) : tf2::TransformException(errorDescription) { ; };
};


}  // namespace tf2
}  // namespace common
}  // namespace openbot

#endif // OPENBOT_COMMON_TF2_EXCEPTIONS_HPP_
