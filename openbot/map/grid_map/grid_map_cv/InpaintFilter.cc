/*
 * InpaintFilter.cpp
 *
 *  Created on: May 6, 2017
 *      Author: Tanja Baumann, Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "openbot/map/grid_map/grid_map_cv/InpaintFilter.hpp"

// Grid Map
#include "openbot/map/grid_map/grid_map_core/grid_map_core.hpp"

#include "glog/logging.h"


namespace grid_map {

// template<typename T>
// InpaintFilter<T>::InpaintFilter()
//     : radius_(5.0) {

// }

// template<typename T>
// InpaintFilter<T>::~InpaintFilter() {

// }

// template<typename T>
// bool InpaintFilter<T>::configure() {
//   if (!FilterBase < T > ::getParam(std::string("radius"), radius_)) {
//     LOG(ERROR) << "InpaintRadius filter did not find param radius.";
//     return false;
//   }

//   if (radius_ < 0.0) {
//     LOG(ERROR) << "Radius must be greater than zero.";
//     return false;
//   }

//   LOG(WARN) << "Radius = ", radius_;

//   if (!FilterBase < T > ::getParam(std::string("input_layer"), inputLayer_)) {
//     LOG(ERROR) << "Inpaint filter did not find parameter `input_layer`.";
//     return false;
//   }

//   LOG(WARN) << "Inpaint input layer is = " << inputLayer_.c_str();

//   if (!FilterBase < T > ::getParam(std::string("output_layer"), outputLayer_)) {
//     LOG(ERROR) << "Inpaint filter did not find parameter `output_layer`.";
//     return false;
//   }

//   LOG(WARN) << "Inpaint output layer = " << outputLayer_.c_str();

//   return true;
// }

// template<typename T>
// bool InpaintFilter<T>::update(const T& mapIn, T& mapOut) {
//   // Add new layer to the elevation map.
//   mapOut = mapIn;
//   mapOut.add(outputLayer_);

//   //Convert elevation layer to OpenCV image to fill in holes.
//   //Get the inpaint mask (nonzero pixels indicate where values need to be filled in).
//   mapOut.add("inpaint_mask", 0.0);

//   mapOut.setBasicLayers(std::vector<std::string>());
//   for (grid_map::GridMapIterator iterator(mapOut); !iterator.isPastEnd(); ++iterator) {
//     if (!mapOut.isValid(*iterator, inputLayer_)) {
//       mapOut.at("inpaint_mask", *iterator) = 1.0;
//     }
//   }
//   cv::Mat originalImage;
//   cv::Mat mask;
//   cv::Mat filledImage;
//   const float minValue = mapOut.get(inputLayer_).minCoeffOfFinites();
//   const float maxValue = mapOut.get(inputLayer_).maxCoeffOfFinites();

//   grid_map::GridMapCvConverter::toImage<unsigned char, 3>(mapOut, inputLayer_, CV_8UC3, minValue, maxValue,
//                                                           originalImage);
//   grid_map::GridMapCvConverter::toImage<unsigned char, 1>(mapOut, "inpaint_mask", CV_8UC1, mask);

//   const double radiusInPixels = radius_ / mapIn.getResolution();
//   cv::inpaint(originalImage, mask, filledImage, radiusInPixels, cv::INPAINT_NS);

//   grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 3>(filledImage, outputLayer_, mapOut, minValue, maxValue);
//   mapOut.erase("inpaint_mask");

//   return true;
// }

}