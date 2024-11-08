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


#ifndef OPENBOT_COMMON_TF2_TIME_CACHE_HPP_
#define OPENBOT_COMMON_TF2_TIME_CACHE_HPP_

#include "openbot/common/tf2/transform_storage.hpp"
#include "openbot/common/tf2/time.hpp"

#include <list>
#include <sstream>
#include <boost/shared_ptr.hpp>

namespace openbot {
namespace common {
namespace tf2 {

typedef std::pair<Time, CompactFrameID> P_TimeAndFrameID;

class TimeCacheInterface
{
public:
  /** \brief Access data from the cache */
  virtual bool getData(Time time, TransformStorage & data_out, std::string* error_str = 0)=0; //returns false if data unavailable (should be thrown as lookup exception

  /** \brief Insert data into the cache */
  virtual bool insertData(const TransformStorage& new_data)=0;

  /** @brief Clear the list of stored values */
  virtual void clearList()=0;

  /** \brief Retrieve the parent at a specific time */
  virtual CompactFrameID getParent(Time time, std::string* error_str) = 0;

  /**
   * \brief Get the latest time stored in this cache, and the parent associated with it.  Returns parent = 0 if no data.
   */
  virtual P_TimeAndFrameID getLatestTimeAndParent() = 0;


  /// Debugging information methods
  /** @brief Get the length of the stored list */
  virtual unsigned int getListLength()=0;

  /** @brief Get the latest timestamp cached */
  virtual Time getLatestTimestamp()=0;

  /** @brief Get the oldest timestamp cached */
  virtual Time getOldestTimestamp()=0;
};

typedef boost::shared_ptr<TimeCacheInterface> TimeCacheInterfacePtr;

/** \brief A class to keep a sorted linked list in time
 * This builds and maintains a list of timestamped
 * data.  And provides lookup functions to get
 * data out as a function of time. */
class TimeCache : public TimeCacheInterface
{
 public:
  static const int MIN_INTERPOLATION_DISTANCE = 5; //!< Number of nano-seconds to not interpolate below.
  static const unsigned int MAX_LENGTH_LINKED_LIST = 1000000; //!< Maximum length of linked list, to make sure not to be able to use unlimited memory.
  static const int64_t DEFAULT_MAX_STORAGE_TIME = 1ULL * 1000000000LL; //!< default value of 10 seconds storage

  TimeCache(Duration  max_storage_time = DEFAULT_MAX_STORAGE_TIME);


  /// Virtual methods

  virtual bool getData(Time time, TransformStorage & data_out, std::string* error_str = 0);
  virtual bool insertData(const TransformStorage& new_data);
  virtual void clearList();
  virtual CompactFrameID getParent(Time time, std::string* error_str);
  virtual P_TimeAndFrameID getLatestTimeAndParent();

  /// Debugging information methods
  virtual unsigned int getListLength();
  virtual Time getLatestTimestamp();
  virtual Time getOldestTimestamp();
  

private:
  typedef std::list<TransformStorage> L_TransformStorage;
  L_TransformStorage storage_;

  Duration max_storage_time_;


  /// A helper function for getData
  //Assumes storage is already locked for it
  inline uint8_t findClosest(TransformStorage*& one, TransformStorage*& two, Time target_time, std::string* error_str);

  inline void interpolate(const TransformStorage& one, const TransformStorage& two, Time time, TransformStorage& output);


  void pruneList();



};

class StaticCache : public TimeCacheInterface
{
 public:
  /// Virtual methods

  virtual bool getData(Time time, TransformStorage & data_out, std::string* error_str = 0); //returns false if data unavailable (should be thrown as lookup exception
  virtual bool insertData(const TransformStorage& new_data);
  virtual void clearList();
  virtual CompactFrameID getParent(Time time, std::string* error_str);
  virtual P_TimeAndFrameID getLatestTimeAndParent();


  /// Debugging information methods
  virtual unsigned int getListLength();
  virtual Time getLatestTimestamp();
  virtual Time getOldestTimestamp();
  

private:
  TransformStorage  storage_;
};

}  // namespace tf2
}  // namespace common
}  // namespace openbot

#endif // OPENBOT_COMMON_TF2_TIME_CACHE_HPP_
