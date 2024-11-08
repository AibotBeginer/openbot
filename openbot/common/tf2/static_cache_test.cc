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

#include "openbot/common/tf2/time_cache.hpp"

#include <gtest/gtest.h>
#include <sys/time.h>
#include <stdexcept>
#include <cmath>

namespace openbot {
namespace common {
namespace tf2 {


void setIdentity(TransformStorage& stor)
{
  stor.translation_.setValue(0.0, 0.0, 0.0);
  stor.rotation_.setValue(0.0, 0.0, 0.0, 1.0);
}

TEST(StaticCache, Repeatability)
{
  unsigned int runs = 100;
  
  tf2::StaticCache  cache;

  TransformStorage stor;
  setIdentity(stor);
  
  for ( uint64_t i = 1; i < runs ; i++ )
  {
    stor.frame_id_ = CompactFrameID(i);
    stor.stamp_ = tf2::Time(i);
    
    cache.insertData(stor);

    
    cache.getData(tf2::Time(i), stor);
    EXPECT_EQ(stor.frame_id_, i);
    EXPECT_EQ(stor.stamp_, tf2::Time(i));
    
  }
}

TEST(StaticCache, DuplicateEntries)
{

  tf2::StaticCache cache;

  TransformStorage stor;
  setIdentity(stor);
  stor.frame_id_ = CompactFrameID(3);
  stor.stamp_ = tf2::Time(1);

  cache.insertData(stor);

  cache.insertData(stor);


  cache.getData(tf2::Time(1), stor);
  
  //printf(" stor is %f\n", stor.transform.translation.x);
  EXPECT_TRUE(!std::isnan(stor.translation_.x()));
  EXPECT_TRUE(!std::isnan(stor.translation_.y()));
  EXPECT_TRUE(!std::isnan(stor.translation_.z()));
  EXPECT_TRUE(!std::isnan(stor.rotation_.x()));
  EXPECT_TRUE(!std::isnan(stor.rotation_.y()));
  EXPECT_TRUE(!std::isnan(stor.rotation_.z()));
  EXPECT_TRUE(!std::isnan(stor.rotation_.w()));
}

}  // namespace tf2
}  // namespace common
}  // namespace openbot
