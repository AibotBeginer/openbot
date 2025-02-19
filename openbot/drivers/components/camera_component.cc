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

#include "openbot/drivers/components/camera_component.hpp"
#include "openbot/common/utils/logging.hpp"

namespace openbot {
namespace drivers {
namespace components {

bool CameraComponent::Init() 
{
  camera_config_ = std::make_shared<openbot::drivers::camera::config::Config>();
  if (!apollo::cyber::common::GetProtoFromFile(config_file_path_, camera_config_.get())) 
  {
    return false;
  }

  AINFO << "Sensor camera output with: " << camera_config_->width();
  AINFO << "Sensor camera output height: " << camera_config_->height();

  writer_ = node_->CreateWriter<openbot_bridge::common_msgs::Image>(camera_config_->channel_name());
  raw_writer_ = node_->CreateWriter<openbot_bridge::common_msgs::Image>(camera_config_->raw_channel_name());
  ros2_msgs_writer_ = node_->CreateWriter<openbot_bridge::common_msgs::Image>(camera_config_->raw_channel_name());

  async_result_ = apollo::cyber::Async(&CameraComponent::run, this);
  return true;
}

void CameraComponent::run() 
{
  std::string imagePath = "/workspace/openbot/src/1.jpg";
  cv::Mat image = cv::imread(imagePath, cv::IMREAD_COLOR);

  // 检查图像是否成功加载
  if (image.empty()) {
    AERROR << "Failed to load image: " << imagePath;
    return;
  }

  running_.exchange(true);
  while (!apollo::cyber::IsShutdown()) {

    auto raw_image = std::make_shared<openbot_bridge::common_msgs::Image>();
    ToImage(image, *raw_image.get());
    raw_writer_->Write(raw_image);

    // auto raw_image = std::make_shared<openbot_bridge::ros2_msgs::sensor_msgs::Image>();
    // ToImage(image, *raw_image.get());
    // ros2_msgs_writer_->Write(raw_image);
    
    LOG(INFO) << "Publish images";
    apollo::cyber::SleepFor(std::chrono::seconds(1));
  }
}

void CameraComponent::ToImage(const cv::Mat& cvImage, openbot_bridge::common_msgs::Image& image)
{
  auto header_time = apollo::cyber::Time::Now().ToSecond();
  image.mutable_header()->set_frame_id("test");
  image.set_height(cvImage.rows);
  image.set_width(cvImage.cols);
  image.set_step(cvImage.step);

  // 设置编码格式
  // OpenCV 的常见编码格式有 "bgr8", "rgb8", "mono8", 等等
  if (cvImage.type() == CV_8UC3) {
      image.set_encoding("bgr8");
  } else if (cvImage.type() == CV_8UC1) {
      image.set_encoding("mono8");
  }
  // 其他类型可以根据需要添加

  // 设置图像数据
  size_t dataSize = cvImage.step * cvImage.rows;
  image.set_data(cvImage.data, dataSize);
}

CameraComponent::~CameraComponent() 
{
  if (running_.load()) {
    running_.exchange(false);
    // free(raw_image_->image);
    async_result_.wait();
  }
}

}  // namespace components
}  // namespace drivers
}  // namespace openbot
