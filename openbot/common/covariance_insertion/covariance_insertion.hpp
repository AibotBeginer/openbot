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


#ifndef OPENBOT_COMMON_COVARIANCE_INSERTION__COVARIANCE_INSERTION_HPP_
#define OPENBOT_COMMON_COVARIANCE_INSERTION__COVARIANCE_INSERTION_HPP_

#include "openbot/common/helper_functions/types.hpp"
#include "openbot/common/covariance_insertion/add_covariance.hpp"


#include <map>
#include <string>
#include <vector>

namespace openbot {
namespace common {
namespace covariance_insertion {

/// @brief Class for performing covariance insertion
class CovarianceInsertion
{
public:
  /// @brief      constructor, creates a map
  CovarianceInsertion();

  /// @brief      populate msg from the covarianes
  /// @param      msg  message to be populated
  template<typename MsgT>
  void set_all_covariances(MsgT * msg)
  {
    if (!msg) {return;}
    for (const auto & kv : m_covariances) {
      const auto & field = kv.first;
      const auto & covariance = kv.second;
      add_covariance(msg, covariance, field);
    }
  }

  /// @brief      check if the covariance map is empty
  bool covariances_empty();

  /// @brief      maps covariance to field
  /// @param      field  map key
  /// @param      covariance  map value
  void insert_covariance(
    const std::string & field,
    const std::vector<common::types::float64_t> & covariance);

private:
  /// Map of covariance values
  std::map<std::string, std::vector<common::types::float64_t>> m_covariances;
};

}  // namespace covariance_insertion
}  // namespace common
}  // namespace openbot

#endif  // OPENBOT_COMMON_COVARIANCE_INSERTION__COVARIANCE_INSERTION_HPP_
