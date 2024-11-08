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


#include <gtest/gtest.h>
#include <sys/time.h>

#include "openbot/common/tf2/LinearMath/Vector3.hpp"
#include "openbot/common/tf2/exceptions.hpp"
#include "openbot/common/tf2/buffer_core.hpp"

namespace openbot {
namespace common {
namespace tf2 {


TEST(tf2, setTransformFail)
{
  tf2::BufferCore tfc;
  geometry_msgs::TransformStamped st;
  EXPECT_FALSE(tfc.setTransform(st, "authority1"));

}

TEST(tf2, setTransformValid)
{
  tf2::BufferCore tfc;
  geometry_msgs::TransformStamped st;
  st.header.frame_id = "foo";
  st.header.stamp = tf2::Time(1e9);
  st.child_frame_id = "child";
  st.transform.rotation.w = 1;
  EXPECT_TRUE(tfc.setTransform(st, "authority1"));

}

TEST(tf2, setTransformInvalidQuaternion)
{
  tf2::BufferCore tfc;
  geometry_msgs::TransformStamped st;
  st.header.frame_id = "foo";
  st.header.stamp = tf2::Time(1e9);
  st.child_frame_id = "child";
  st.transform.rotation.w = 0;
  EXPECT_FALSE(tfc.setTransform(st, "authority1"));

}

TEST(tf2_lookupTransform, LookupException_Nothing_Exists)
{
  tf2::BufferCore tfc;
  EXPECT_THROW(tfc.lookupTransform("a", "b", tf2::Time(1e9)), tf2::LookupException);

}

TEST(tf2_canTransform, Nothing_Exists)
{
  tf2::BufferCore tfc;
  EXPECT_FALSE(tfc.canTransform("a", "b", tf2::Time(1e9)));

  std::string error_msg = std::string();
  EXPECT_FALSE(tfc.canTransform("a", "b", tf2::Time(1e9), &error_msg));
  ASSERT_STREQ(error_msg.c_str(), "canTransform: target_frame a does not exist. canTransform: source_frame b does not exist.");

}

TEST(tf2_lookupTransform, LookupException_One_Exists)
{
  tf2::BufferCore tfc;
  geometry_msgs::TransformStamped st;
  st.header.frame_id = "foo";
  st.header.stamp = tf2::Time(1e9);
  st.child_frame_id = "child";
  st.transform.rotation.w = 1;
  EXPECT_TRUE(tfc.setTransform(st, "authority1"));
  EXPECT_THROW(tfc.lookupTransform("foo", "bar", tf2::Time(1e9)), tf2::LookupException);

}

TEST(tf2_canTransform, One_Exists)
{
  tf2::BufferCore tfc;
  geometry_msgs::TransformStamped st;
  st.header.frame_id = "foo";
  st.header.stamp = tf2::Time(1e9);
  st.child_frame_id = "child";
  st.transform.rotation.w = 1;
  EXPECT_TRUE(tfc.setTransform(st, "authority1"));
  EXPECT_FALSE(tfc.canTransform("foo", "bar", tf2::Time(1e9)));
}


}  // namespace tf2
}  // namespace common
}  // namespace openbot