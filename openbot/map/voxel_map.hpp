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

#ifndef OPENBOT_MAP_VOXEL_MAP_HPP_
#define OPENBOT_MAP_VOXEL_MAP_HPP_

#include "openbot/common/port.hpp"
#include "openbot/common/macros.hpp"
#include "openbot/map/costmap.hpp"
#include "openbot/map/voxel_dilater.hpp"

#include <memory>
#include <vector>
#include <Eigen/Eigen>

namespace openbot {
namespace map { 

class VoxelMap : public Costmap
{
public:
    /**
     *  @brief SharedPtr typedef
     */
    OPENBOT_SMART_PTR_DEFINITIONS(VoxelMap);

    /**
     * @brief A default constructor for openbot::map::VoxelMap
     */
    VoxelMap() = default;

    /**
     * @brief A constructor for openbot::map::VoxelMap
     * @param size Map size
     * @param origin
     * @param voxScale
     */
    VoxelMap(const Eigen::Vector3i& size, const Eigen::Vector3d& origin, const double& vox_scale);

    /**
     * @brief Get scale value
     */
    Eigen::Vector3i GetSize(void) const
    {
        return map_size_;
    }

    /**
     * @brief Get scale value
     */
    double GetScale() const
    {
        return scale_;
    }

    /**
     * @brief Get value
     */
    Eigen::Vector3d GetOrigin() const
    {
        return o_;
    }

    /**
     * @brief Get value
     */
    inline Eigen::Vector3d GetCorner() const
    {
        return map_size_.cast<double>() * scale_ + o_;
    }

    /**
     * @brief Get value
     */
    const std::vector<uint8>& GetVoxels() const
    {
        return voxels_;
    }

    /**
     * @brief Get value
     */
    void SetOccupied(const Eigen::Vector3d& pos);

    /**
     * @brief Get value
     */
    void SetOccupied(const Eigen::Vector3i& id);

    /**
     * @brief Get value
     */
    void Dilate(const int &r);

    /**
     * @brief Get value
     */
    void GetSurfInBox(const Eigen::Vector3i& center, const int& half_width, std::vector<Eigen::Vector3d>& points) const;

    /**
     * @brief Get 
     */
    void GetSurf(std::vector<Eigen::Vector3d>& points) const;

    /**
     * @brief Get 
     */
    bool Query(const Eigen::Vector3d& pos) const;

    /**
     * @brief Get 
     */
    bool Query(const Eigen::Vector3i& id) const;

    /**
     * @brief Get 
     */
    inline Eigen::Vector3d Cast(const Eigen::Vector3i& id) const
    {
        return id.cast<double>() * scale_ + oc_;
    }

    /**
     * @brief Get 
     */
    inline Eigen::Vector3i Cast(const Eigen::Vector3d& pos) const
    {
        return ((pos - o_) / scale_).cast<int>();
    }

private:
    Eigen::Vector3i map_size_;
    Eigen::Vector3d o_;
    double scale_;
    int vox_num_;
    Eigen::Vector3i step_;
    Eigen::Vector3d oc_;
    Eigen::Vector3i bounds_;
    Eigen::Vector3d step_scale_;
    std::vector<uint8> voxels_;
    std::vector<Eigen::Vector3i> surf_;
};

}  // namespace map
}  // namespace openbot

#endif  // OPENBOT_MAP_VOXEL_MAP_HPP_
