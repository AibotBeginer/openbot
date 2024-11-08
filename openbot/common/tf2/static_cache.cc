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


/** \author Tully Foote */

#include "openbot/common/tf2/time_cache.hpp"
#include "openbot/common/tf2/exceptions.hpp"
#include "openbot/common/tf2/LinearMath/Transform.hpp"

namespace openbot {
namespace common {
namespace tf2 {


bool StaticCache::getData(Time time, TransformStorage & data_out, std::string* error_str) //returns false if data not available
{
  data_out = storage_;
  data_out.stamp_ = time;
  (void)error_str;
  return true;
};

bool StaticCache::insertData(const TransformStorage& new_data)
{
  storage_ = new_data;
  return true;
};


void StaticCache::clearList() { return; };

unsigned int StaticCache::getListLength() {   return 1; };

CompactFrameID StaticCache::getParent(Time time, std::string* error_str)
{
  (void)time;
  (void)error_str;
  return storage_.frame_id_;
}

P_TimeAndFrameID StaticCache::getLatestTimeAndParent()
{
  return std::make_pair(Time(), storage_.frame_id_);
}

Time StaticCache::getLatestTimestamp() 
{   
  return Time();
};

Time StaticCache::getOldestTimestamp() 
{   
  return Time();
};

}  // namespace tf2
}  // namespace common
}  // namespace openbot
