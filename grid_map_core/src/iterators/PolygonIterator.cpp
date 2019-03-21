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

PolygonIterator::PolygonIterator(grid_map::GridMap& gridMap, const grid_map::Polygon& polygon)
    : polygon_(polygon.getVertices())
{
  gridMap.convertToDefaultStartIndex();
  mapLength_ = gridMap.getLength();
  mapPosition_ = gridMap.getPosition();
  resolution_ = gridMap.getResolution();
  bufferSize_ = gridMap.getSize();
  bufferStartIndex_ = gridMap.getStartIndex();
  // TODO: Make this adaptive to vertex bounding box.
  minInd_ = convertPositionToIndexSpace(polygon.getMaxCoords()).cast<int>();
  maxInd_ = convertPositionToIndexSpace(polygon.getMinCoords()).cast<int>();
  minInd_.x() = std::min(std::max(0, minInd_.x()), bufferSize_.x());
  minInd_.y() = std::min(std::max(0, minInd_.y()), bufferSize_.y());
  maxInd_.x() = std::max(std::min(bufferSize_.x(), maxInd_.x()+1), 1);
  maxInd_.y() = std::max(std::min(bufferSize_.y(), maxInd_.y()+1), 1);
  // Convert polygons to index space.
  convertVerticesToIndexSpace();
  // If we have an incomplete polygon, iterate past end immediately.
  if (polygon_.size() < 3) {
    isPastEnd_ = true;
  } else {
    buildEdgeTables();
    updateActiveEdges();
    iterateUntilInside();
  }
}

bool PolygonIterator::operator !=(const PolygonIterator& other) const
{
  return (curInd_.x() != other.curInd_.x() && curInd_.y() != other.curInd_.y());
}

const Index& PolygonIterator::operator *() const
{
  return curInd_;
}

PolygonIterator& PolygonIterator::operator ++()
{
  incrementIndices();
  iterateUntilInside();

  return *this;
}

void PolygonIterator::iterateUntilInside() {
  for (; !isPastEnd(); incrementIndices()) {
    // Check all edges we passed.
    for (; curActiveEdgeIter_ != activeEdgeTable_.end() && curActiveEdgeIter_->yHit <= curInd_.y(); ++curActiveEdgeIter_) {
      unevenEdgeHits_ = !unevenEdgeHits_;
    }
    if (unevenEdgeHits_) break;
  }

}

void PolygonIterator::updateActiveEdges()
{
  // Update yHit.
  for (auto& edge: activeEdgeTable_) {
    if (!std::isinf(edge.yHit)) {
      edge.yHit += edge.mInv;
    }
  }
  // Get newly active edges from edgeTable_.
  for (auto&& edge_iter = edgeTable_.begin();
       edge_iter != edgeTable_.end();) {
    if (edge_iter->xMin <= curInd_.x()) {
      // Compute correct first yHit.
      if (!std::isinf(edge_iter->mInv)) {
        edge_iter->yHit += (curInd_.x() - edge_iter->xMin) * edge_iter->mInv;
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
    if (edge_iter->xMax < curInd_.x()) {
      edge_iter = activeEdgeTable_.erase(edge_iter);
    } else {
      ++edge_iter;
    }
  }
  // Reset active edge iterator.
  curActiveEdgeIter_ = activeEdgeTable_.begin();
}

void PolygonIterator::incrementIndices()
{
  ++curInd_.y();
  if (curInd_.y() == maxInd_.y()) {
    curInd_.y() = minInd_.y();
    ++curInd_.x();
    if (checkPastEnd()) return;
  }
  // Check for new line.
  if (curLineX_ != curInd_.x()) {
    curLineX_ = curInd_.x();
    updateActiveEdges();
    unevenEdgeHits_ = false;
  }
}

void PolygonIterator::convertVerticesToIndexSpace() {
  for (auto& position: polygon_) {
    position = convertPositionToIndexSpace(position);
  }
}

inline Position PolygonIterator::convertPositionToIndexSpace(const Position& position) {
  return -((position - (0.5 * (mapLength_ - resolution_)).matrix() - mapPosition_).array() / resolution_).matrix();
}

void PolygonIterator::buildEdgeTables()
{
  size_t maxXInd, minXind;
  Edge edge;
  for (size_t curVert = 0, nextVert = 1; curVert < polygon_.size(); ++curVert, ++nextVert) {
    // Close polygon with first element if we reached the last vertex.
    if (nextVert == polygon_.size()) nextVert = 0;
    // Figure out which vertex has the lower x value.
    if (Polygon::sortVertices(polygon_[curVert], polygon_[nextVert])) {
      minXind = curVert;
      maxXInd = nextVert;
    } else {
      minXind = nextVert;
      maxXInd = curVert;
    }
    const auto& minVert = polygon_[minXind];
    const auto& maxVert = polygon_[maxXInd];
    // Compute edge.
    edge.xMin = minVert.x();
    edge.xMax = maxVert.x();
    edge.yHit = minVert.y();
    const auto dx = maxVert.x() - minVert.x();
    const auto dy = maxVert.y() - minVert.y();
    edge.mInv = (dx < std::numeric_limits<double>::epsilon()) ? std::numeric_limits<double>::infinity() : dy/dx;
    edgeTable_.push_back(edge);
  }
  // Sort edge table.
  edgeTable_.sort([](const Edge& edge1, const Edge& edge2) {
    return (edge1.xMin < edge2.xMin) || (edge1.xMin == edge2.xMin && edge1.yHit < edge2.yHit);
  });

}

} /* namespace grid_map */

