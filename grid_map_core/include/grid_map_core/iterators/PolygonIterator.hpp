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
  PolygonIterator(const grid_map::GridMap& gridMap, const grid_map::Polygon& polygon);

  /*!
   * Assignment operator.
   * @param iterator the iterator to copy data from.
   * @return a reference to *this.
   */
  PolygonIterator& operator =(const PolygonIterator& other);

  /*!
   * Compare to another iterator.
   * @return whether the current iterator points to a different address than the other one.
   */
  bool operator !=(const PolygonIterator& other) const;

  /*!
   * Dereference the iterator with const.
   * @return the value to which the iterator is pointing.
   */
  const Index& operator *() const;

  /*!
   * Increase the iterator to the next element.
   * @return a reference to the updated iterator.
   */
  PolygonIterator& operator ++();

  /*!
   * Indicates if iterator is past end.
   * @return true if iterator is out of scope, false if end has not been reached.
   */
  bool isPastEnd() const;

private:

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
      return edge1.yHit > edge2.yHit;
    }

   public:

//    void sortedInsert(const Edge& edge) {
//      insert(std::lower_bound(begin(), end(), edge, edgeComp), edge);
//    }

    void sortedInsert(Edge&& edge) {
      insert(std::lower_bound(begin(), end(), edge, edgeComp), std::move(edge));
    }
  };

  /*!
   * Check if current index is inside polygon.
   * @return true if inside, false otherwise.
   */
  bool isInside() const;

  /*!
   * Finds the submap that fully contains the polygon and returns the parameters.
   * @param[in] polygon the polygon to get the submap for.
   * @param[out] startIndex the start index of the submap.
   * @param[out] bufferSize the buffer size of the submap.
   */
  void findSubmapParameters(const grid_map::Polygon& polygon, Index& startIndex,Size& bufferSize) const;

  void buildEdgeTables();

  void updateActiveEdges();

  void updateIndex();

  void updatePosition();

  void updateIndexAndPosition();

  bool incrementInternalIterator();

  //! Polygon to iterate on.
  grid_map::Polygon polygon_;

  //! Grid submap iterator.
  std::shared_ptr<SubmapIterator> internalIterator_;

  //! Map information needed to get position from iterator.
  Length mapLength_;
  Position mapPosition_;
  double resolution_;
  Size bufferSize_;
  Index bufferStartIndex_;

  //! Scanline algorithm variables.
  std::list<Edge> edgeTable_;
  SortedEdgeList activeEdgeTable_;
  Index curIndex_;
  Position curPosition_;
  double curLineX_{std::numeric_limits<double>::max()};
  bool unevenEdgeHits_{false};
  SortedEdgeList::iterator curActiveEdgeIter_;


 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} /* namespace */
