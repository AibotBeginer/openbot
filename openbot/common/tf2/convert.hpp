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


#ifndef OPENBOT_COMMON_TF2_CONVERT_HPP_
#define OPENBOT_COMMON_TF2_CONVERT_HPP_

#include "openbot/common/tf2/transform_datatypes.hpp"
#include "openbot/common/tf2/exceptions.hpp"
#include "openbot/common/tf2/impl/convert.hpp"
#include "openbot/common/tf2/time.hpp"
#include "openbot/common/msgs/msgs.hpp"

namespace openbot {
namespace common {
namespace tf2 {


/**\brief The templated function expected to be able to do a transform
 *
 * This is the method which tf2 will use to try to apply a transform for any given datatype.   
 * \param data_in The data to be transformed.
 * \param data_out A reference to the output data.  Note this can point to data in and the method should be mutation safe.
 * \param transform The transform to apply to data_in to fill data_out.  
 * 
 * This method needs to be implemented by client library developers
 */
template <class T>
  void doTransform(const T& data_in, T& data_out, const geometry_msgs::TransformStamped& transform);

/**\brief Get the timestamp from data 
 * \param t The data input.
 * \return The timestamp associated with the data. The lifetime of the returned
 * reference is bound to the lifetime of the argument.
 */
template <class T>
  const Time& getTimestamp(const T& t);

/**\brief Get the frame_id from data 
 * \param t The data input.
 * \return The frame_id associated with the data. The lifetime of the returned
 * reference is bound to the lifetime of the argument.
 */
template <class T>
  const std::string& getFrameId(const T& t);



/* An implementation for Stamped<P> datatypes */
template <class P>
  const Time& getTimestamp(const tf2::Stamped<P>& t)
  {
    return t.stamp_;
  }

/* An implementation for Stamped<P> datatypes */
template <class P>
  const std::string& getFrameId(const tf2::Stamped<P>& t)
  {
    return t.frame_id_;
  }

/** Function that converts from one type to a ROS message type. It has to be
 * implemented by each data type in tf2_* (except ROS messages) as it is
 * used in the "convert" function.
 * \param a an object of whatever type
 * \return the conversion as a ROS message
 */
template<typename A, typename B>
  B toMsg(const A& a);

/** Function that converts from a ROS message type to another type. It has to be
 * implemented by each data type in tf2_* (except ROS messages) as it is used
 * in the "convert" function.
 * \param a a ROS message to convert from
 * \param b the object to convert to
 */
template<typename A, typename B>
  void fromMsg(const A&, B& b);

/** Function that converts any type to any type (messages or not).
 * Matching toMsg and from Msg conversion functions need to exist.
 * If they don't exist or do not apply (for example, if your two
 * classes are ROS messages), just write a specialization of the function.
 * \param a an object to convert from
 * \param b the object to convert to
 */
template <class A, class B>
  void convert(const A& a, B& b)
  {
    //printf("In double type convert\n");
    // impl::Converter<ros::message_traits::IsMessage<A>::value, ros::message_traits::IsMessage<B>::value>::convert(a, b);
  }

template <class A>
  void convert(const A& a1, A& a2)
  {
    //printf("In single type convert\n");
    if(&a1 != &a2)
      a2 = a1;
  }


}  // namespace tf2
}  // namespace common
}  // namespace openbot

#endif // OPENBOT_COMMON_TF2_CONVERT_HPP_
