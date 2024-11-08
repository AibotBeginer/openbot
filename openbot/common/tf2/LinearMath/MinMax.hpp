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


#ifndef GEN_MINMAX_HPP_
#define GEN_MINMAX_HPP_

namespace openbot {
namespace common {
namespace tf2 {


template <class T>
TF2SIMD_FORCE_INLINE const T& tf2Min(const T& a, const T& b) 
{
  return a < b ? a : b ;
}

template <class T>
TF2SIMD_FORCE_INLINE const T& tf2Max(const T& a, const T& b) 
{
  return  a > b ? a : b;
}

template <class T>
TF2SIMD_FORCE_INLINE const T& GEN_clamped(const T& a, const T& lb, const T& ub) 
{
	return a < lb ? lb : (ub < a ? ub : a); 
}

template <class T>
TF2SIMD_FORCE_INLINE void tf2SetMin(T& a, const T& b) 
{
    if (b < a) 
	{
		a = b;
	}
}

template <class T>
TF2SIMD_FORCE_INLINE void tf2SetMax(T& a, const T& b) 
{
    if (a < b) 
	{
		a = b;
	}
}

template <class T>
TF2SIMD_FORCE_INLINE void GEN_clamp(T& a, const T& lb, const T& ub) 
{
	if (a < lb) 
	{
		a = lb; 
	}
	else if (ub < a) 
	{
		a = ub;
	}
}

}  // namespace tf2
}  // namespace common
}  // namespace openbot

#endif
