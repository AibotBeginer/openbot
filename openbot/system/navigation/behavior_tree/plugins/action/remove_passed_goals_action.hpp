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

#include <string>
#include <memory>


#include "behaviortree_cpp/action_node.h"
#include "openbot/common/io/msgs.hpp"
#include "openbot/system/navigation/behavior_tree/bt_service_node.hpp"
#include "openbot/system/navigation/proto/get_costs.pb.h"

                                
namespace openbot {
namespace system {
namespace navigation {
namespace behavior_tree {

class RemovePassedGoals : public BT::ActionNodeBase
{
public:
    typedef std::vector<common::geometry_msgs::PoseStamped> Goals;

    RemovePassedGoals(
        const std::string& xml_tag_name,
        const BT::NodeConfiguration& conf);

    /**
     * @brief Function to read parameters and initialize class variables
     */
    void Initialize();

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<Goals>("input_goals", "Original goals to remove viapoints from"),
            BT::OutputPort<Goals>("output_goals", "Goals with passed viapoints removed"),
            BT::InputPort<double>("radius", 0.5, "radius to goal for it to be considered for removal"),
            BT::InputPort<std::string>("global_frame", "Global frame"),
            BT::InputPort<std::string>("robot_base_frame", "Robot base frame"),
        };
    }

private:
    void halt() override {}
    BT::NodeStatus tick() override;

    double viapoint_achieved_radius_;
    double transform_tolerance_;
    // std::shared_ptr<tf2_ros::Buffer> tf_;
    std::string robot_base_frame_;
    bool initialized_;
};

}   // namespace behavior_tree 
}   // namespace navigation
}   // namespace system
}   // namespace openbot