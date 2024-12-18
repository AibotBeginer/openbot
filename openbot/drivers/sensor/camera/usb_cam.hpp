// /*
//  * Copyright 2024 The OpenRobotic Beginner Authors
//  *
//  * Licensed under the Apache License, Version 2.0 (the "License");
//  * you may not use this file except in compliance with the License.
//  * You may obtain a copy of the License at
//  *
//  *      http://www.apache.org/licenses/LICENSE-2.0
//  *
//  * Unless required by applicable law or agreed to in writing, software
//  * distributed under the License is distributed on an "AS IS" BASIS,
//  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  * See the License for the specific language governing permissions and
//  * limitations under the License.
//  */


// #pragma once

// #include <asm/types.h> /* for videodev2.h */
// #include <malloc.h>
// #include <sys/ioctl.h>
// #include <sys/mman.h>

// #ifndef __aarch64__
// #include <immintrin.h>
// #include <x86intrin.h>
// #endif

// extern "C" {
// #include <libavcodec/avcodec.h>
// #include <libavutil/mem.h>
// #include <libswscale/swscale.h>
// #include <linux/videodev2.h>
// }

// #include <libavcodec/version.h>
// #if LIBAVCODEC_VERSION_MAJOR < 55
// #define AV_CODEC_ID_MJPEG CODEC_ID_MJPEG
// #endif

// #include <memory>
// #include <sstream>
// #include <string>

// #include "cyber/cyber.h"

// #include "openbot/drivers/proto/camera_config.pb.h"

// namespace openbot {
// namespace drivers {
// namespace sensor {
// namespace camera {

// using openbot::drivers::camera::config::Config;
// using openbot::drivers::camera::config::IO_METHOD_MMAP;
// using openbot::drivers::camera::config::IO_METHOD_READ;
// using openbot::drivers::camera::config::IO_METHOD_UNKNOWN;
// using openbot::drivers::camera::config::IO_METHOD_USERPTR;
// using openbot::drivers::camera::config::RGB;
// using openbot::drivers::camera::config::YUYV;

// // camera raw image struct
// struct CameraImage 
// {
//   int width;
//   int height;
//   int bytes_per_pixel;
//   int image_size;
//   int is_new;
//   int tv_sec;
//   int tv_usec;
//   char* image;

//   ~CameraImage() {
//     if (image != nullptr) {
//       free(reinterpret_cast<void*>(image));
//       image = nullptr;
//     }
//   }
// };

// typedef std::shared_ptr<CameraImage> CameraImagePtr;

// struct buffer 
// {
//   void* start;
//   size_t length;
// };

// class UsbCam 
// {
// public:
//   UsbCam();
//   virtual ~UsbCam();

//   virtual bool init(const std::shared_ptr<Config>& camera_config);
//   // user use this function to get camera frame data
//   virtual bool poll(const CameraImagePtr& raw_image, const CameraImagePtr& sensor_raw_image);

//   bool is_capturing();
//   bool wait_for_device(void);

// private:
//   int xioctl(int fd, int request, void* arg);
//   bool init_device(void);
//   bool uninit_device(void);

//   void set_device_config();
//   // enables/disable auto focus
//   void set_auto_focus(int value);
//   // set video device parameters
//   void set_v4l_parameter(const std::string& param, int value);
//   void set_v4l_parameter(const std::string& param, const std::string& value);

//   int init_mjpeg_decoder(int image_width, int image_height);
//   void mjpeg2rgb(char* mjepg_buffer, int len, char* rgb_buffer, int pixels);
// #ifdef __aarch64__
//   int convert_yuv_to_rgb_pixel(int y, int u, int v);
//   int convert_yuv_to_rgb_buffer(unsigned char* yuv, unsigned char* rgb,
//                                 unsigned int width, unsigned int height);
// #endif

//   bool init_read(unsigned int buffer_size);
//   bool init_mmap(void);
//   bool init_userp(unsigned int buffer_size);
//   bool set_adv_trigger(void);
//   bool close_device(void);
//   bool open_device(void);
//   bool read_frame(CameraImagePtr raw_image, CameraImagePtr sensor_raw_image);
//   bool process_image(void* src, int len, CameraImagePtr dest);
//   bool start_capturing(void);
//   bool stop_capturing(void);
//   void reconnect();
//   void reset_device();

//   std::shared_ptr<Config> config_;
//   int pixel_format_;
//   int fd_;
//   buffer* buffers_;
//   unsigned int n_buffers_;
//   bool is_capturing_;
//   uint64_t image_seq_;

//   AVFrame* avframe_camera_;
//   AVFrame* avframe_rgb_;
//   AVCodec* avcodec_;
//   AVDictionary* avoptions_;
//   AVCodecContext* avcodec_context_;
//   int avframe_camera_size_;
//   int avframe_rgb_size_;
//   struct SwsContext* video_sws_;

//   float frame_warning_interval_ = 0.0;
//   float device_wait_sec_ = 0.0;
//   uint64_t last_nsec_ = 0;
//   float frame_drop_interval_ = 0.0;
// };

// }  // namespace camera
// }  // namespace sensor
// }  // namespace drivers
// }  // namespace oepnbot
