
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


#include "openbot/map/components/map_component.hpp"
#include "openbot/common/utils/logging.hpp"

namespace openbot {
namespace map { 

bool MapComponent::Init() 
{
    LOG(INFO) << "Load MapComponent ....";
    map_config_ = std::make_shared<MapConfig>();
    if (!apollo::cyber::common::GetProtoFromFile(config_file_path_, map_config_.get())) {
      return false;
    }
    LOG(INFO) << "Filter config: " << map_config_->DebugString();

    LOG(INFO) << map_config_->frame_id();
    LOG(INFO) << map_config_->channel_name();
    LOG(INFO) << map_config_->ply_path();
    
    map_server_ = std::make_shared<MapServer>();

    map_writer_ = node_->CreateWriter<openbot_bridge::sensor_msgs::PointCloud>(map_config_->channel_name());
    async_result_ = apollo::cyber::Async(&MapComponent::Run, this);
    return true;
}

void MapComponent::Run() 
{
  running_.exchange(true);
  while (!apollo::cyber::IsShutdown()) {
    auto map_data = std::make_shared<openbot_bridge::sensor_msgs::PointCloud>();
    map_writer_->Write(map_data);

    LOG(INFO) << "Publish ply pointcloud";
    apollo::cyber::SleepFor(std::chrono::seconds(1));
  }
}

MapComponent::~MapComponent()
{
  if (running_.load()) {
    running_.exchange(false);
    // free(raw_image_->image);
    async_result_.wait();
  }
}

}  // namespace map 
}  // namespace openbot