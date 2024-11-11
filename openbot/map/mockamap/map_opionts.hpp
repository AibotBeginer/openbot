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

#ifndef OPENBOT_MAP_MOCKAMAP_MAP_OPIONTS_HPP
#define OPENBOT_MAP_MOCKAMAP_MAP_OPIONTS_HPP

#include "openbot/common/port.hpp"

namespace openbot {
namespace map { 
namespace mockamap { 

struct MapOption
{
    //  box edge length, unit meter
    double resolution;
    uint32 seed;
    double scale;
    uint32 x_length;
    uint32 y_length;
    uint32 z_length;
    uint32 type;    // 1 perlin noise 3D
                    // 2 perlin box random map
                    // 3 2d maze 
                    // 4 3d maze
    /**
    type = 1 perlin noise parameters
        complexity:    base noise frequency, large value will be complex typical 0.0 ~ 0.5
        fill:          infill persentage typical: 0.4 ~ 0.0 
        fractal:       large value will have more detail
        attenuation:   for fractal attenuation typical: 0.0 ~ 0.5

    complexity = 0.03;
    fill = 0.3;
    fractal = 1;
    attenuation = 0.1;
    */
    double complexity;
    double fill;
    double attenuation;
    uint32 fractal;

    /**
    type = 2  perlin box random map
        width_min = 0.6;
        width_max = 1.5;
        obstacle_number = 50;
    */
    double width_min;
    double width_max;
    uint32 obstacle_number;

    /**
    type = 3  2d maze 
        road_width = 0.5;
        add_wall_x = 0;
        add_wall_y = 1;
        maze_type = 1;
    */
    double road_width;
    uint32 add_wall_x;
    uint32 add_wall_y;
    uint32 maze_type;  // 1 recursive division maze

    /**
    type = 4  3d maze 
        num_nodes = 40;
        connectivity = 0.8;
        node_rad = 1;
        road_rad = 10;
    */
    uint32 num_nodes;
    double connectivity;
    uint32 node_rad;
    uint32 road_rad;

    // Constructor for Perlin Noise 3D
    MapOption(double res, uint32_t x, uint32_t y, uint32_t z, double comp, double fill, double atten, uint32_t frac)
        : resolution(res), x_length(x), y_length(y), z_length(z), type(1),
          complexity(comp), fill(fill), attenuation(atten), fractal(frac) 
    {}

    // Constructor for Perlin Box Random Map
    MapOption(double res, uint32_t x, uint32_t y, uint32_t z, double wmin, double wmax, uint32_t obs_num)
        : resolution(res), x_length(x), y_length(y), z_length(z), type(2),
          width_min(wmin), width_max(wmax), obstacle_number(obs_num) 
    {}

    // Constructor for 2D Maze
    MapOption(double res, uint32_t x, uint32_t y, double road_w, uint32_t add_x, uint32_t add_y, uint32_t maze_t)
        : resolution(res), x_length(x), y_length(y), z_length(0), type(3),
          road_width(road_w), add_wall_x(add_x), add_wall_y(add_y), maze_type(maze_t) 
    {}

    // Constructor for 3D Maze
    MapOption(double res, uint32_t x, uint32_t y, uint32_t z, uint32_t nodes, double conn, uint32_t node_r, uint32_t road_r)
        : resolution(res), x_length(x), y_length(y), z_length(z), type(4),
          num_nodes(nodes), connectivity(conn), node_rad(node_r), road_rad(road_r) 
    {}

    // Constructor with default values
    MapOption()
        : seed(511), resolution(0.1),
          x_length(10), y_length(10), z_length(3), type(1),
          complexity(0.03), fill(0.3), fractal(1), attenuation(0.1),
          width_min(0.6), width_max(1.5), obstacle_number(50),
          road_width(0.5), add_wall_x(0), add_wall_y(1), maze_type(1),
          num_nodes(40), connectivity(0.8), node_rad(1), road_rad(10) 
    {}
};

}  // namespace mockamap
}  // namespace map
}  // namespace openbot

#endif // OPENBOT_MAP_MOCKAMAP_MAP_OPIONTS_HPP
