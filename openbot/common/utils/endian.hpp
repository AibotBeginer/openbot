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

#include <algorithm>
#include <iostream>
#include <vector>

namespace openbot {
namespace common {
namespace utils {

// Reverse the order of each byte.
template <typename T>
T ReverseBytes(const T& data);

// Check the order in which bytes are stored in computer memory.
bool IsLittleEndian();
bool IsBigEndian();

// Convert data between endianness and the native format. Note that, for float
// and double types, these functions are only valid if the format is IEEE-754.
// This is the case for pretty much most processors.
template <typename T>
T LittleEndianToNative(T x);
template <typename T>
T BigEndianToNative(T x);
template <typename T>
T NativeToLittleEndian(T x);
template <typename T>
T NativeToBigEndian(T x);

// Read data in little endian format for cross-platform support.
template <typename T>
T ReadBinaryLittleEndian(std::istream* stream);
template <typename T>
void ReadBinaryLittleEndian(std::istream* stream, std::vector<T>* data);

// Write data in little endian format for cross-platform support.
template <typename T>
void WriteBinaryLittleEndian(std::ostream* stream, const T& data);
template <typename T>
void WriteBinaryLittleEndian(std::ostream* stream, const std::vector<T>& data);

////////////////////////////////////////////////////////////////////////////////
// Implementation
////////////////////////////////////////////////////////////////////////////////

template <typename T>
T ReverseBytes(const T& data) {
  T data_reversed = data;
  std::reverse(reinterpret_cast<char*>(&data_reversed),
               reinterpret_cast<char*>(&data_reversed) + sizeof(T));
  return data_reversed;
}

template <typename T>
T LittleEndianToNative(const T x) {
  if (IsLittleEndian()) {
    return x;
  } else {
    return ReverseBytes(x);
  }
}

template <typename T>
T BigEndianToNative(const T x) {
  if (IsBigEndian()) {
    return x;
  } else {
    return ReverseBytes(x);
  }
}

template <typename T>
T NativeToLittleEndian(const T x) {
  if (IsLittleEndian()) {
    return x;
  } else {
    return ReverseBytes(x);
  }
}

template <typename T>
T NativeToBigEndian(const T x) {
  if (IsBigEndian()) {
    return x;
  } else {
    return ReverseBytes(x);
  }
}

template <typename T>
T ReadBinaryLittleEndian(std::istream* stream) {
  T data_little_endian;
  stream->read(reinterpret_cast<char*>(&data_little_endian), sizeof(T));
  return LittleEndianToNative(data_little_endian);
}

template <typename T>
void ReadBinaryLittleEndian(std::istream* stream, std::vector<T>* data) {
  for (size_t i = 0; i < data->size(); ++i) {
    (*data)[i] = ReadBinaryLittleEndian<T>(stream);
  }
}

template <typename T>
void WriteBinaryLittleEndian(std::ostream* stream, const T& data) {
  const T data_little_endian = NativeToLittleEndian(data);
  stream->write(reinterpret_cast<const char*>(&data_little_endian), sizeof(T));
}

template <typename T>
void WriteBinaryLittleEndian(std::ostream* stream, const std::vector<T>& data) {
  for (const auto& elem : data) {
    WriteBinaryLittleEndian<T>(stream, elem);
  }
}

}  // namespace utils
}  // namespace common
}  // namespace openbot
