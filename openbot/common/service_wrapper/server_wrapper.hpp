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

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <future>
#include <chrono>
#include <functional>

#include "cyber/service/service.h"
#include "openbot/common/utils/logging.hpp"

/**
 * @namespace openbot::common
 * @brief openbot::common
 */
namespace openbot {
namespace common {

/**
 * @brief Wrapper of cyber::Client which sends a topic with service name when
 * SendRequst is invoked.
 */
template <class ServiceT>
class ServiceWrapper 
{
public:

    using ExecuteCallback = std::function<void ()>;

    using CompletionCallback = std::function<void ()>;

    ServiceWrapper(const std::shared_ptr<::apollo::cyber::Node>& node,
                   const std::string& service_name,
                   ExecuteCallback execute_callback,
                   CompletionCallback completion_callback = nullptr,
                   std::chrono::milliseconds server_timeout = std::chrono::milliseconds(500))
    {
        
        accept_server_ = node->CreateService<typename ServiceT::Request, typename ServiceT::Response>(
            service_name,
            [this](const std::shared_ptr<typename ServiceT::Request>& request, 
                   std::shared_ptr<typename ServiceT::Response>& response) {
                HandleAccepted(request, response);
            });

        cancel_server_  = node->CreateService<typename ServiceT::Request, typename ServiceT::Response>(
            service_name,
            [](const std::shared_ptr<typename ServiceT::Request>& request,
               std::shared_ptr<typename ServiceT::Response>& response) {
                HandleCancel(request, response);
            });
    }

    /**
     * @brief 
     * 
     * @param request 
     * @return true 
     * @return false 
     */
    bool HandleAccepted(
        const std::shared_ptr<typename ServiceT::Request>& request, 
        std::shared_ptr<typename ServiceT::Response>& response)
    { 
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        InfoMsg("Receiving a new goal");

        if (IsActive(current_handle_) || IsRunning()) {
            InfoMsg("An older goal is active, moving the new goal to a pending slot.");

            if (IsActive(pending_handle_)) {
                InfoMsg(
                "The pending slot is occupied. The previous pending goal will be terminated and replaced.");
                Terminate(pending_handle_);
            }
            pending_handle_ = handle;
            preempt_requested_ = true;
        } else {
            if (IsActive(pending_handle_)) {
                // Shouldn't reach a state with a pending goal but no current one.
                ErrorMsg("Forgot to handle a preemption. Terminating the pending goal.");
                Terminate(pending_handle_);
                preempt_requested_ = false;
            }

            current_handle_ = handle;

            // Return quickly to avoid blocking the executor, so spin up a new thread
            InfoMsg("Executing goal asynchronously.");
            accept_execution_future_ = std::async(std::launch::async, [this]() {AcceptedWork();});
        }
        return true;
    }

    /**
     * @brief 
     * 
     * @param request 
     * @return ServiceT::Response 
     */
    bool HandleCancel(
        const std::shared_ptr<typename ServiceT::Request>& request, 
        std::shared_ptr<typename ServiceT::Response>& response)
    {
        // std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        // if (!handle->is_active()) {
        //     WarnMsg("Received request for goal cancellation, but the handle is inactive, so reject the request");
        //     return typename ServiceT::Response::Status::REJECT;
        // }

        // InfoMsg("Received request for goal cancellation");
        // return typename ServiceT::Response::Status::ACCEPT;

        InfoMsg("Executing goal asynchronously.");
        cancel_execution_future_ = std::async(std::launch::async, [this]() {CanceledWork();});

        return true;
    }

    /**
     * @brief Computed background work and processes stop requests
     */
    void AcceptedWork()
    {
        while (apollo::cyber::Ok() && !stop_execution_ && IsActive(current_handle_)) 
        {
            InfoMsg("Executing the goal...");
            try {
                execute_callback_();
            } catch (std::exception& ex) {
                LOG(ERROR) << "Action server failed while executing action callback: " << ex.what();
                TerminateAll();
                completion_callback_();
                return;
            }

            InfoMsg("Blocking processing of new goal handles.");
            std::lock_guard<std::recursive_mutex> lock(update_mutex_);

            if (stop_execution_) {
                WarnMsg("Stopping the thread per request.");
                TerminateAll();
                completion_callback_();
                break;
            }

            if (IsActive(current_handle_)) {
                WarnMsg("Current goal was not completed successfully.");
                Terminate(current_handle_);
                completion_callback_();
            }

            if (IsActive(pending_handle_)) {
                InfoMsg("Executing a pending handle on the existing thread.");
                AcceptPendingRequest();
            } else {
                InfoMsg("Done processing available goals.");
                break;
            }
        }
        InfoMsg("Accept worker thread done.");
    }

    void CanceledWork()
    {
        while (apollo::cyber::Ok() && !stop_execution_ && IsActive(current_handle_)) 
        {

        }
        InfoMsg("Cancel worker thread done.");
    }

    /**
     * @brief Whether the action server is munching on a goal
     * @return bool If its running or not
     */
    bool IsRunning()
    {
        return accept_execution_future_.valid() &&
            (accept_execution_future_.wait_for(std::chrono::milliseconds(0)) ==
            std::future_status::timeout);
    }

    /**
     * @brief Whether the action server is active or not
     * @return bool If its active or not
     */
    bool IsServerActive()
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        return server_active_;
    }

    /**
     * @brief Whether the action server has been asked to be preempted with a new goal
     * @return bool If there's a preemption request or not
     */
    bool IsPreemptRequested() const
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        return preempt_requested_;
    }

     /**
     * @brief Terminate pending request
     */
    void TerminatePendingRequest()
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        if (!pending_handle_ || !pending_handle_->is_active()) {
            ErrorMsg("Attempting to terminate pending request when not available");
            return;
        }

        Terminate(pending_handle_);
        preempt_requested_ = false;
        InfoMsg("Pending goal terminated");
    }

    /**
     * @brief Get the current goal object
     * @return Goal Ptr to the  goal that's being processed currently
     */
    const std::shared_ptr<const typename ActionT::Request> GetCurrentRequest() const
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);

        if (!IsActive(current_handle_)) {
            ErrorMsg("A goal is not available or has reached a final state");
            return std::shared_ptr<const typename ActionT::Request>();
        }

        return current_handle_->get_goal();
    }

    /**
     * @brief Get the pending goal object
     * @return Goal Ptr to the goal that's pending
     */
    const std::shared_ptr<const typename ServiceT::Request> GetPendingRequest() const
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);

        if (!pending_handle_ || !pending_handle_->is_active()) {
            ErrorMsg("Attempting to get pending goal when not available");
            return std::shared_ptr<const typename ServiceT::Goal>();
        }

        return pending_handle_->get_goal();
    }

    /**
     * @brief Whether or not a cancel command has come in
     * @return bool Whether a cancel command has been requested or not
     */
    bool IsCancelRequested() const
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);

        // A cancel request is assumed if either handle is canceled by the client.

        if (current_handle_ == nullptr) {
            ErrorMsg("Checking for cancel but current goal is not available");
            return false;
        }

        if (pending_handle_ != nullptr) {
            return pending_handle_->is_canceling();
        }

        return current_handle_->is_canceling();
    }

    /**
     * @brief Terminate all pending and active actions
     * @param result A result object to send to the terminated actions
     */
    void TerminateAll(
        typename std::shared_ptr<typename ServiceT::Response> result =
        std::make_shared<typename ServiceT::Response>())
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        Terminate(current_handle_, result);
        Terminate(pending_handle_, result);
        preempt_requested_ = false;
    }


    /**
     * @brief Terminate the active action
     * @param result A result object to send to the terminated action
     */
    void TerminateCurrent(
        typename std::shared_ptr<typename ServiceT::Response> result =
        std::make_shared<typename ServiceT::Response>())
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        Terminate(current_handle_, result);
    }

    /**
     * @brief Return success of the active action
     * @param result A result object to send to the terminated actions
     */
    void SucceededCurrent(
        typename std::shared_ptr<typename ServiceT::Response> result =
        std::make_shared<typename ServiceT::Response>())
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);

        if (is_active(current_handle_)) {
            InfoMsg("Setting succeed on current goal.");
            current_handle_->succeed(result);
            current_handle_.reset();
        }
    }

protected:
    /**
     * @brief Info logging
     */
    void InfoMsg(const std::string& msg) const
    {
        LOG(INFO) <<  service_name_.c_str() << "[ActionServer] " <<  msg.c_str();
    }

    /**
     * @brief Error logging
     */
    void ErrorMsg(const std::string& msg) const
    {
        LOG(ERROR) <<  service_name_.c_str() << "[ActionServer] " <<  msg.c_str();
    }

    /**
     * @brief Warn logging
     */
    void WarnMsg(const std::string& msg) const
    {
        LOG(WARNING) <<  service_name_.c_str() << "[ActionServer] " <<  msg.c_str();
    }

    /**
     * @brief Generate an empty result object for an action type
     */
    constexpr auto EmptyResult() const
    {
        return std::make_shared<typename ServiceT::Response>();
    }

    /**
     * @brief Terminate a particular action with a result
     * @param handle goal handle to terminate
     * @param the Results object to terminate the action with
     */
    void Terminate(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<ServiceT>> & handle,
        typename std::shared_ptr<typename ServiceT::Response> result =
        std::make_shared<typename ServiceT::Response>())
    {
        std::lock_guard<std::recursive_mutex> lock(update_mutex_);
        if (IsActive(handle)) {
            if (handle->IsCanceling()) {
                WarnMsg("Client requested to cancel the goal. Cancelling.");
                handle->canceled(result);
            } else {
                WarnMsg("Aborting handle.");
                handle->abort(result);
            }
            handle.reset();
        }
    }

    std::string service_name_;

    ExecuteCallback execute_callback_;
    CompletionCallback completion_callback_;
    std::future<void> accept_execution_future_;
    std::future<void> cancel_execution_future_;
    bool stop_execution_{false};

    mutable std::recursive_mutex update_mutex_;
    bool server_active_{false};
    bool preempt_requested_{false};
    std::chrono::milliseconds server_timeout_;

    std::shared_ptr<::apollo::cyber::Service<typename ServiceT::Request, typename ServiceT::Response>> current_handle_;
    std::shared_ptr<::apollo::cyber::Service<typename ServiceT::Request, typename ServiceT::Response>> pending_handle_;
    std::shared_ptr<::apollo::cyber::Service<typename ServiceT::Request, typename ServiceT::Response>> accept_server_;
    std::shared_ptr<::apollo::cyber::Service<typename ServiceT::Request, typename ServiceT::Response>> cancel_server_;
};

}  // namespace common
}  // namespace openbot
