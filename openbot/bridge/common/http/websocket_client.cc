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

#include "openbot/bridge/common/http/websocket_client.hpp"

#include <chrono>

#include "cyber/common/log.h"
#include "cyber/cyber.h"

namespace openbot {
namespace bridge { 
namespace http {

WebsocketClient::WebsocketClient(const std::string& host, const std::string& port)
 : host_(host), port_(port)
{

    resolver_ = std::make_shared<boost::asio::ip::tcp::resolver>(ioc_);
    ws_ = std::make_shared<boost::beast::websocket::stream<tcp::socket>>(ioc_);
}

bool WebsocketClient::Connect()
{
    try {
        // Resolve the host name
        auto const results = resolver_->resolve(host_, port_);

        // Make the connection on the IP address we get from a lookup
        boost::asio::connect(ws_->next_layer(), results.begin(), results.end());

        // Perform the WebSocket handshake
        ws_->handshake(host_, "/");
        connect_finished_ = true;
        return true;
    } catch (std::exception const& e) {
        std::cerr << "Connect error: " << e.what() << std::endl;
        return false;
    }

    return true;
}

bool WebsocketClient::Send(const std::string& message)
{
    return ws_->write(boost::asio::buffer(message));
}

std::string WebsocketClient::Receive()
{
    try {
        boost::beast::flat_buffer buffer;
        ws_->read(buffer);
        return boost::beast::buffers_to_string(buffer.data());
    } catch (std::exception const& e) {
        std::cerr << "Receive error: " << e.what() << std::endl;
        return "";
    }
}

void WebsocketClient::Close()
{
    try {
        ws_->close(boost::beast::websocket::close_code::normal);
    } catch (std::exception const& e) {
        std::cerr << "Close error: " << e.what() << std::endl;
    }

    connect_finished_ = false;
}

}  // namespace http 
}  // namespace bridge 
}  // namespace openbot
