/*
 * GridMapLoader.hpp
 *
 *  Created on: Aug 24, 2015
 *      Author: Peter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 *
 */

#pragma once

#include <string>

#include "openbot/map/grid_map/grid_map_msgs/grid_map.hpp"

namespace grid_map_loader {

/*!
 * Loads and publishes a grid map from a bag file.
 */
class GridMapLoader
{
 public:

  /*!
   * Constructor.
   */
  GridMapLoader();

  /*!
   * Destructor.
   */
  virtual ~GridMapLoader();

  /*!
  * Reads and verifies the ROS parameters.
  * @return true if successful.
  */
  bool readParameters();

  /*!
   * Loads the grid map from the bag file.
   * @return true if successful, false otherwise.
   */
  bool load();

  /*!
   * Publishes the grid map.
   */
  grid_map::grid_map_msgs::GridMap& map_data();

 private:

  //! Grid map data.
  grid_map::grid_map_msgs::GridMap map_;

  //! Path the ROS bag to be published.
  std::string filePath_;
};

} /* namespace */
