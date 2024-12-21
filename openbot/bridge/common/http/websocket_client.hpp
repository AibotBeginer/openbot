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


#include <iostream>
#include <string>
#include <memory>

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>

#include "openbot_bridge/ros2_msgs/sensor_msgs.pb.h"

namespace openbot {
namespace bridge { 
namespace http { 

using tcp = boost::asio::ip::tcp;               // From <boost/asio/ip/tcp.hpp>

class WebsocketClient 
{
public:
    /**
     *  @brief SharedPtr typedef
     */
    OPENBOT_SMART_PTR_DEFINITIONS(WebsocketClient);

     /**
     * @brief construct function
     */
    WebsocketClient() = default;

    /**
     * @brief construct function
     * @param host The websocket ip addr
     * @param port  The websocket ip addr's port
     */
    WebsocketClient(const std::string& host, const std::string& port);

    /**
     * @brief Destructor for openbot::bridge::http::WebsocketClient
     */
    ~WebsocketClient() = default;

    /**
     * @brief connect
     * 
     */
    void Connect();

    /**
     * @brief Send msgs
     * 
     */
    void Send(const std::string& message);

    /**
     * @brief Receive msgs
     * 
     * @return std::string 
     */
    std::string Receive();

    /**
     * @brief Close connect
     * 
     */
    void Close();

    /**
     * @brief function that send msg through websocket
     * @param input msgs The type msg
     */
    template <typename T>
    void SendRosTopicMessages(const T&& msg);

private:
    // boost web
    boost::asio::io_context ioc_;
    std::shared_ptr<boost::asio::ip::tcp::resolver> resolver_{nullptr};
    std::shared_ptr<boost::beast::websocket::stream<tcp::socket>> ws_{nullptr};
    std::string host_;
    std::string port_;
};
}  // namespace http 
}  // namespace bridge 
}  // namespace openbot
