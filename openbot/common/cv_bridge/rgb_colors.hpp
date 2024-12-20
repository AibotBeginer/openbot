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

#include <opencv2/opencv.hpp>

namespace openbot {
namespace common {
namespace cv_bridge {
namespace rgb_colors {

/**
 * @brief
 * 146 rgb colors
 */
enum Colors
{
  ALICEBLUE,
  ANTIQUEWHITE,
  AQUA,
  AQUAMARINE,
  AZURE,
  BEIGE,
  BISQUE,
  BLACK,
  BLANCHEDALMOND,
  BLUE,
  BLUEVIOLET,
  BROWN,
  BURLYWOOD,
  CADETBLUE,
  CHARTREUSE,
  CHOCOLATE,
  CORAL,
  CORNFLOWERBLUE,
  CORNSILK,
  CRIMSON,
  CYAN,
  DARKBLUE,
  DARKCYAN,
  DARKGOLDENROD,
  DARKGRAY,
  DARKGREEN,
  DARKGREY,
  DARKKHAKI,
  DARKMAGENTA,
  DARKOLIVEGREEN,
  DARKORANGE,
  DARKORCHID,
  DARKRED,
  DARKSALMON,
  DARKSEAGREEN,
  DARKSLATEBLUE,
  DARKSLATEGRAY,
  DARKSLATEGREY,
  DARKTURQUOISE,
  DARKVIOLET,
  DEEPPINK,
  DEEPSKYBLUE,
  DIMGRAY,
  DIMGREY,
  DODGERBLUE,
  FIREBRICK,
  FLORALWHITE,
  FORESTGREEN,
  FUCHSIA,
  GAINSBORO,
  GHOSTWHITE,
  GOLD,
  GOLDENROD,
  GRAY,
  GREEN,
  GREENYELLOW,
  GREY,
  HONEYDEW,
  HOTPINK,
  INDIANRED,
  INDIGO,
  IVORY,
  KHAKI,
  LAVENDER,
  LAVENDERBLUSH,
  LAWNGREEN,
  LEMONCHIFFON,
  LIGHTBLUE,
  LIGHTCORAL,
  LIGHTCYAN,
  LIGHTGOLDENRODYELLOW,
  LIGHTGRAY,
  LIGHTGREEN,
  LIGHTGREY,
  LIGHTPINK,
  LIGHTSALMON,
  LIGHTSEAGREEN,
  LIGHTSKYBLUE,
  LIGHTSLATEGRAY,
  LIGHTSLATEGREY,
  LIGHTSTEELBLUE,
  LIGHTYELLOW,
  LIME,
  LIMEGREEN,
  LINEN,
  MAGENTA,
  MAROON,
  MEDIUMAQUAMARINE,
  MEDIUMBLUE,
  MEDIUMORCHID,
  MEDIUMPURPLE,
  MEDIUMSEAGREEN,
  MEDIUMSLATEBLUE,
  MEDIUMSPRINGGREEN,
  MEDIUMTURQUOISE,
  MEDIUMVIOLETRED,
  MIDNIGHTBLUE,
  MINTCREAM,
  MISTYROSE,
  MOCCASIN,
  NAVAJOWHITE,
  NAVY,
  OLDLACE,
  OLIVE,
  OLIVEDRAB,
  ORANGE,
  ORANGERED,
  ORCHID,
  PALEGOLDENROD,
  PALEGREEN,
  PALEVIOLETRED,
  PAPAYAWHIP,
  PEACHPUFF,
  PERU,
  PINK,
  PLUM,
  POWDERBLUE,
  PURPLE,
  RED,
  ROSYBROWN,
  ROYALBLUE,
  SADDLEBROWN,
  SALMON,
  SANDYBROWN,
  SEAGREEN,
  SEASHELL,
  SIENNA,
  SILVER,
  SKYBLUE,
  SLATEBLUE,
  SLATEGRAY,
  SLATEGREY,
  SNOW,
  SPRINGGREEN,
  STEELBLUE,
  TAN,
  TEAL,
  THISTLE,
  TOMATO,
  TURQUOISE,
  VIOLET,
  WHEAT,
  WHITE,
  WHITESMOKE,
  YELLOW,
  YELLOWGREEN,
};

/**
 * @brief
 * get rgb color with enum.
 */
cv::Vec3d getRGBColor(const int color);

}  // namespace rgb_colors
}  // namespace cv_bridge
}  // namespace openbot
}  // namespace common