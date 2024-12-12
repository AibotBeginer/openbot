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


#ifndef OPENBOT_PLANNING_GLOBAL_PLANNER_SERVERHPP_
#define OPENBOT_PLANNING_GLOBAL_PLANNER_SERVERHPP_

#include <memory>
#include <string>
#include <unordered_map>

#include "cyber/common/file.h"
#include "cyber/plugin_manager/plugin_manager.h"

#include "openbot/common/macros.hpp"
#include "openbot/map/voxel_map.hpp"
#include "openbot/map/costmap.hpp"
#include "openbot/planning/common/global_planner.hpp"
#include "openbot/planning/plugins/rrt_planner.hpp"
#include "openbot/planning/plugins/a_star_planner.hpp"
#include "openbot/planning/proto/global_planner.pb.h"

// #include "openbot/common/proto/nav_msgs/path.pb.h"
// #include "openbot/common/proto/geometry_msgs/pose_stamped.pb.h"

// #include "class_loader/class_loader.hpp"

#include "cyber/cyber.h"

namespace openbot {
namespace planning { 

class PlannerServer
{
public:
    using PlannerMap = std::unordered_map<std::string, GlobalPlanner::SharedPtr>;

    /**
     *  @brief SharedPtr typedef
     */
    OPENBOT_SMART_PTR_DEFINITIONS(PlannerServer);

    /**
     * @brief A constructor for openbot::planning::PlannerServer
     * @param options Additional options to control creation of the node.
     */
    explicit PlannerServer();

    /**
     * @brief Destructor for openbot::planning::PlannerServer
     */
    ~PlannerServer();

    /**
     * @brief Init server's source
     */
    void Configure();

    template <typename T>
    bool LoadConfig(const std::string& custom_config_path, T* config);

    /**
     * @brief Method to get plan from the desired plugin
     * @param start starting pose
     * @param goal goal request
     * @return Path
     */
    common::nav_msgs::Path GetPlan(
        const common::geometry_msgs::PoseStamped& start,
        const common::geometry_msgs::PoseStamped& goal,
        const std::string & planner_id);
        
    void InitMap(const map::Costmap::SharedPtr costmap);

    void SetRunningPlanner(const std::string& name);
    
    std::string running_planner_name();

    common::nav_msgs::Path CreatePlan(
        const common::geometry_msgs::PoseStamped& start,
        const common::geometry_msgs::PoseStamped& goal, const double timeout);

private:

    /**
     * @brief Handle make plan service request callback
     * @param request The service request cmd
     * @param response The service response result
     */
    void HandleMakePlanServiceCallback(const std::shared_ptr<proto::MakePlanResquest>& request, 
        std::shared_ptr<proto::MakePlanResponse>& response);

    /**
     * @brief The server callback which calls planner to get the path
     * ComputePathToPose
     */
    void ComputePlan();

    // Planner
    PlannerMap planners_;
    
    // pluginlib::ClassLoader<nav2_core::GlobalPlanner> gp_loader_;
    std::vector<std::string> default_ids_;
    std::vector<std::string> default_types_;
    std::vector<std::string> planner_ids_;
    std::vector<std::string> planner_types_;
    double max_planner_duration_;
    std::string planner_ids_concat_;

    std::string running_planner_name_;

    map::Costmap::SharedPtr costmap_{nullptr};

    // Cyber Node
    std::unique_ptr<apollo::cyber::Node> node_{nullptr};

    // pb config
    proto::PlannerConfig planner_conf_;
};

template <typename T>
bool PlannerServer::LoadConfig(const std::string& custom_config_path, T* config) 
{
  std::string config_path = custom_config_path;
  // Get the default config file if "custom_config_path" is empty.
  if ("" == config_path) {
    int status;
    // Get the name of this class.
    std::string class_name = abi::__cxa_demangle(typeid(*this).name(), 0, 0, &status);
    config_path = apollo::cyber::plugin_manager::PluginManager::Instance()->GetPluginConfPath<PlannerServer>(
        class_name, "conf/planning_config.pb.txt");
  }
  return apollo::cyber::common::LoadConfig<T>(config_path, config);
}

}  // namespace planning 
}  // namespace openbot

#endif  // OPENBOT_PLANNING_GLOBAL_PLANNER_SERVERHPP_
