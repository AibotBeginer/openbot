/*
 * EllipseIterator.hpp
 *
 *  Created on: Dec 2, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "openbot/map/grid_map/grid_map_core/iterators/EllipseIterator.hpp"
#include "openbot/map/grid_map/grid_map_core/GridMapMath.hpp"

#include <cmath>
#include <Eigen/Geometry>

namespace grid_map {

EllipseIterator::EllipseIterator(const GridMap& gridMap, const Position& center, const Length& length, const double rotation)
    : center_(center)
{
  semiAxisSquare_ = (0.5 * length).square();
  double sinRotation = std::sin(rotation);
  double cosRotation = std::cos(rotation);
  transformMatrix_ << cosRotation, sinRotation, sinRotation, -cosRotation;
  mapLength_ = gridMap.getLength();
  mapPosition_ = gridMap.getPosition();
  resolution_ = gridMap.getResolution();
  bufferSize_ = gridMap.getSize();
  bufferStartIndex_ = gridMap.getStartIndex();
  Index submapStartIndex;
  Index submapBufferSize;
  findSubmapParameters(center, length, rotation, submapStartIndex, submapBufferSize);
  internalIterator_ = std::make_shared<SubmapIterator>(gridMap, submapStartIndex, submapBufferSize);
  if (!isInside()) {
    ++(*this);
  }
}

bool EllipseIterator::operator !=(const EllipseIterator& other) const
{
  return (internalIterator_ != other.internalIterator_);
}

const Eigen::Array2i& EllipseIterator::operator *() const
{
  return *(*internalIterator_);
}

EllipseIterator& EllipseIterator::operator ++()
{
  ++(*internalIterator_);
  if (internalIterator_->isPastEnd()) {
    return *this;
  }

  for ( ; !internalIterator_->isPastEnd(); ++(*internalIterator_)) {
    if (isInside()) {
      break;
    }
  }

  return *this;
}

bool EllipseIterator::isPastEnd() const
{
  return internalIterator_->isPastEnd();
}

const Size& EllipseIterator::getSubmapSize() const
{
  return internalIterator_->getSubmapSize();
}

bool EllipseIterator::isInside() const
{
  Position position;
  getPositionFromIndex(position, *(*internalIterator_), mapLength_, mapPosition_, resolution_, bufferSize_, bufferStartIndex_);
  double value = ((transformMatrix_ * (position - center_)).array().square() / semiAxisSquare_).sum();
  return (value <= 1);
}

void EllipseIterator::findSubmapParameters(const Position& center, const Length& length, const double rotation,
                                           Index& startIndex, Size& bufferSize) const
{
  const Eigen::Rotation2Dd rotationMatrix(rotation);
  Eigen::Vector2d u = rotationMatrix * Eigen::Vector2d(length(0), 0.0);
  Eigen::Vector2d v = rotationMatrix * Eigen::Vector2d(0.0, length(1));
  const Length boundingBoxHalfLength = (u.cwiseAbs2() + v.cwiseAbs2()).array().sqrt();
  Position topLeft = center.array() + boundingBoxHalfLength;
  Position bottomRight = center.array() - boundingBoxHalfLength;
  boundPositionToRange(topLeft, mapLength_, mapPosition_);
  boundPositionToRange(bottomRight, mapLength_, mapPosition_);
  getIndexFromPosition(startIndex, topLeft, mapLength_, mapPosition_, resolution_, bufferSize_, bufferStartIndex_);
  Index endIndex;
  getIndexFromPosition(endIndex, bottomRight, mapLength_, mapPosition_, resolution_, bufferSize_, bufferStartIndex_);
  bufferSize = getSubmapSizeFromCornerIndices(startIndex, endIndex, bufferSize_, bufferStartIndex_);
}

} /* namespace grid_map */

