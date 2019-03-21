/*
 * PolygonIterator.hpp
 *
 *  Created on: Sep 19, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/Polygon.hpp"
#include "grid_map_core/iterators/SubmapIterator.hpp"

#include <memory>
#include <list>

namespace grid_map {

/*!
 * Iterator class to iterate through a polygonal area of the map.
 */
class PolygonIterator
{
public:

  /*!
   * Constructor.
   * @param gridMap the grid map to iterate on.
   * @param polygon the polygonal area to iterate on.
   */
  PolygonIterator(grid_map::GridMap& gridMap, const grid_map::Polygon& polygon);

  /*!
   * Compare to another iterator.
   * @return whether the current iterator points to a different address than the other one.
   */
  bool operator !=(const PolygonIterator& other) const;

  /*!
   * Dereference the iterator with const.
   * @return the value to which the iterator is pointing.
   */
  const Index &operator *() const;

  /*!
   * Increase the iterator to the next element.
   * @return a reference to the updated iterator.
   */
  PolygonIterator& operator ++();

  /*!
   * Indicates if iterator is past end.
   * @return true if iterator is out of scope, false if end has not been reached.
   */
  inline bool isPastEnd() const
  {
    return isPastEnd_;
  }

private:

  inline bool checkPastEnd() {
    if (curInd_.x() == maxInd_.x()) {
      isPastEnd_ = true;
    }
    return isPastEnd_;
  }

  //! Struct which holds all data for the scanline algorithm.
  struct Edge {
    double xMin;
    double xMax;
    double yHit;
    double mInv;
  };

  //! List of edges sorted by yHit.
  class SortedEdgeList : public std::list<Edge> {

    static bool edgeComp(const Edge& edge1, const Edge& edge2) {
      return edge1.yHit < edge2.yHit;
    }

   public:

//    void sortedInsert(const Edge& edge) {
//      insert(std::lower_bound(begin(), end(), edge, edgeComp), edge);
//    }

    void sortedInsert(Edge&& edge) {
      insert(std::lower_bound(begin(), end(), edge, edgeComp), std::move(edge));
    }
  };

  void buildEdgeTables();

  void updateActiveEdges();

  void incrementIndices();

  void iterateUntilInside();

  void convertVerticesToIndexSpace();

  inline Position convertPositionToIndexSpace(const Position& position);

  //! Polygon to iterate on.
  std::vector<grid_map::Position> polygon_;

  //! Grid iterator index.
  Index curInd_{Index::Constant(0)};
  Index minInd_;
  Index maxInd_;
  bool isPastEnd_{false};

  //! Map information needed to get position from iterator.
  Length mapLength_;
  Position mapPosition_;
  double resolution_;
  Size bufferSize_;
  Index bufferStartIndex_;

  //! Scanline algorithm variables.
  std::list<Edge> edgeTable_;
  SortedEdgeList activeEdgeTable_;
  int curLineX_{std::numeric_limits<int>::lowest()};
  bool unevenEdgeHits_{false};
  SortedEdgeList::iterator curActiveEdgeIter_;


 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} /* namespace */
