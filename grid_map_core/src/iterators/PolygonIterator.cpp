/*
 * PolygonIterator.hpp
 *
 *  Created on: Sep 19, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_core/iterators/PolygonIterator.hpp"
#include "grid_map_core/GridMapMath.hpp"

using namespace std;

namespace grid_map {

PolygonIterator::PolygonIterator(const grid_map::GridMap& gridMap, const grid_map::Polygon& polygon)
    : polygon_(polygon)
{
  mapLength_ = gridMap.getLength();
  mapPosition_ = gridMap.getPosition();
  resolution_ = gridMap.getResolution();
  bufferSize_ = gridMap.getSize();
  bufferStartIndex_ = gridMap.getStartIndex();
  Index submapStartIndex;
  Size submapBufferSize;
  findSubmapParameters(polygon, submapStartIndex, submapBufferSize);
  internalIterator_ = std::shared_ptr<SubmapIterator>(new SubmapIterator(gridMap, submapStartIndex, submapBufferSize));
  updateIndexAndPosition();
  // If we have an incomplete polygon, iterate past end immediately.
  if (polygon_.nVertices() < 3) {
    while (!internalIterator_->isPastEnd()) ++(*internalIterator_);
  } else {
    buildEdgeTables();
    updateActiveEdges();
  }
  if(!isInside()) ++(*this);
}

PolygonIterator& PolygonIterator::operator =(const PolygonIterator& other)
{
  polygon_ = other.polygon_;
  internalIterator_ = other.internalIterator_;
  mapLength_ = other.mapLength_;
  mapPosition_ = other.mapPosition_;
  resolution_ = other.resolution_;
  bufferSize_ = other.bufferSize_;
  bufferStartIndex_ = other.bufferStartIndex_;
  return *this;
}

bool PolygonIterator::operator !=(const PolygonIterator& other) const
{
  return (internalIterator_ != other.internalIterator_);
}

const Index& PolygonIterator::operator *() const
{
  return *(*internalIterator_);
}

PolygonIterator& PolygonIterator::operator ++()
{
  incrementInternalIterator();

  for (; !internalIterator_->isPastEnd(); incrementInternalIterator()) {
    // Check all edges we passed.
    for (; curActiveEdgeIter_ != activeEdgeTable_.end() && curActiveEdgeIter_->yHit >= curPosition_.y(); ++curActiveEdgeIter_) {
      unevenEdgeHits_ = !unevenEdgeHits_;
    }
    if (unevenEdgeHits_) break;
  }

  return *this;
}

void PolygonIterator::updateActiveEdges()
{
  // Update yHit.
  for (auto& edge: activeEdgeTable_) {
    if (!std::isinf(edge.yHit)) {
      edge.yHit -= resolution_ * edge.mInv;
    }
  }
  // Get newly active edges from edgeTable_.
  for (auto&& edge_iter = edgeTable_.begin();
       edge_iter != edgeTable_.end();) {
    if (edge_iter->xMax >= curPosition_.x()) {
      // Compute correct first yHit.
      if (!std::isinf(edge_iter->mInv)) {
        edge_iter->yHit -= (edge_iter->xMax - curPosition_.x()) * edge_iter->mInv;
      }
      // Move to active container and remove from edgeTable_.
      activeEdgeTable_.sortedInsert(std::move(*edge_iter));
      edge_iter = edgeTable_.erase(edge_iter);
    } else {
      ++edge_iter;
    }
  }
  // Remove no longer active edges.
  for (auto&& edge_iter = activeEdgeTable_.begin();
       edge_iter != activeEdgeTable_.end();) {
    if (edge_iter->xMin > curPosition_.x()) {
      edge_iter = activeEdgeTable_.erase(edge_iter);
    } else {
      ++edge_iter;
    }
  }
  // Reset active edge iterator.
  curActiveEdgeIter_ = activeEdgeTable_.begin();
}

void PolygonIterator::updateIndex()
{
  curIndex_ = *(*internalIterator_);
}

void PolygonIterator::updatePosition()
{
  getPositionFromIndex(curPosition_, *(*internalIterator_), mapLength_, mapPosition_, resolution_, bufferSize_, bufferStartIndex_);
}

void PolygonIterator::updateIndexAndPosition() {
  updateIndex();
  updatePosition();
}

bool PolygonIterator::incrementInternalIterator()
{
  ++(*internalIterator_);
  if (internalIterator_->isPastEnd()) return false;
  updateIndexAndPosition();
  // Check for new line.
  if (curLineX_ > curPosition_.x()) {
    curLineX_ = curPosition_.x();
    updateActiveEdges();
    unevenEdgeHits_ = false;
    return true;
  }
  return false;
}

bool PolygonIterator::isPastEnd() const
{
  return internalIterator_->isPastEnd();
}

bool PolygonIterator::isInside() const
{
  return polygon_.isInside(curPosition_);
}

void PolygonIterator::findSubmapParameters(const grid_map::Polygon& polygon, Index& startIndex, Size& bufferSize) const
{
  Position topLeft = polygon_.getVertices()[0];
  Position bottomRight = topLeft;
  for (const auto& vertex : polygon_.getVertices()) {
    topLeft = topLeft.array().max(vertex.array());
    bottomRight = bottomRight.array().min(vertex.array());
  }
  boundPositionToRange(topLeft, mapLength_, mapPosition_);
  boundPositionToRange(bottomRight, mapLength_, mapPosition_);
  getIndexFromPosition(startIndex, topLeft, mapLength_, mapPosition_, resolution_, bufferSize_, bufferStartIndex_);
  Index endIndex;
  getIndexFromPosition(endIndex, bottomRight, mapLength_, mapPosition_, resolution_, bufferSize_, bufferStartIndex_);
  bufferSize = getSubmapSizeFromCornerIndeces(startIndex, endIndex, bufferSize_, bufferStartIndex_);
}

void PolygonIterator::buildEdgeTables()
{
  size_t maxXInd, minXind;
  Edge edge;
  for (size_t curVert = 0, nextVert = 1; curVert < polygon_.nVertices(); ++curVert, ++nextVert) {
    // Close polygon with first element if we reached the last vertex.
    if (nextVert == polygon_.nVertices()) nextVert = 0;
    // Figure out which vertex has the lower x value.
    if (Polygon::sortVertices(polygon_.getVertex(curVert), polygon_.getVertex(nextVert))) {
      minXind = curVert;
      maxXInd = nextVert;
    } else {
      minXind = nextVert;
      maxXInd = curVert;
    }
    const auto& minVert = polygon_.getVertex(minXind);
    const auto& maxVert = polygon_.getVertex(maxXInd);
    // Compute edge.
    edge.xMin = minVert.x();
    edge.xMax = maxVert.x();
    edge.yHit = maxVert.y();
    const auto dx = maxVert.x() - minVert.x();
    const auto dy = maxVert.y() - minVert.y();
    edge.mInv = (dx < std::numeric_limits<double>::epsilon()) ? std::numeric_limits<double>::infinity() : dy/dx;
    edgeTable_.push_back(edge);
  }
  // Sort edge table.
  edgeTable_.sort([](const Edge& edge1, const Edge& edge2) {
    return (edge1.xMax > edge2.xMax) || (edge1.xMax == edge2.xMax && edge1.yHit > edge2.yHit);
  });

}

} /* namespace grid_map */

