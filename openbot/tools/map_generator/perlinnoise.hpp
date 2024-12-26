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

#include <vector>

// THIS CLASS IS A TRANSLATION TO C++14 FROM THE REFERENCE
// JAVA IMPLEMENTATION OF THE IMPROVED PERLIN FUNCTION (see
// http://mrl.nyu.edu/~perlin/noise/)
// THE ORIGINAL JAVA IMPLEMENTATION IS COPYRIGHT 2002 KEN PERLIN

// I ADDED AN EXTRA METHOD THAT GENERATES A NEW PERMUTATION VECTOR (THIS IS NOT
// PRESENT IN THE ORIGINAL IMPLEMENTATION)

namespace openbot {
namespace tools {
namespace map_generator { 

class PerlinNoise
{
public:
  // Initialize with the reference values for the permutation vector
  PerlinNoise();
  // Generate a new permutation vector based on the value of seed
  PerlinNoise(unsigned int seed);
  // Get a noise value, for 2D images z can have any value
  double noise(double x, double y, double z);

private:
  double fade(double t);
  double lerp(double t, double a, double b);
  double grad(int hash, double x, double y, double z);

  // The permutation vector
  std::vector<int> p;
};

}  // namespace map_generator
}  // namespace tools
}  // namespace openbot

