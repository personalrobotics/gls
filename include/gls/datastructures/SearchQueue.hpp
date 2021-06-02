/* Author: Aditya Vamsikrishna Mandalika */

#ifndef GLS_DATASTRUCTURES_SEARCHQUEUE_HPP_
#define GLS_DATASTRUCTURES_SEARCHQUEUE_HPP_

// STL headers
#include <functional> // std::function
#include <set>        // std::set

// OMPL headers
#include <ompl/base/Cost.h>
#include <ompl/datastructures/BinaryHeap.h>

// GLS headers
#include "gls/datastructures/Types.hpp"
#include "gls/datastructures/Graph.hpp"

namespace gls {
namespace datastructures {

class SearchQueue {
public:
  /// The function signature of the sorting function for the vertex queue.
  typedef std::function<bool(
      const std::pair<gls::datastructures::Vertex, double>&,
      const std::pair<gls::datastructures::Vertex, double>&)>
      VertexSortingFunction;

  /// The underlying vertex queue.
  typedef std::set<std::pair<gls::datastructures::Vertex, double>, VertexSortingFunction>
      VertexQueue;

  /// Constructor.
  SearchQueue();

  /// Destructor.
  virtual ~SearchQueue() = default;

  /// Clear the search queue.
  void clear();

  /// Adds vertex and value to search queue.
  /// \param[in] vertex Vertex to remove from the queue.
  /// \param[in] cost Cost the vertex is ties to.
  void addVertexWithValue(gls::datastructures::Vertex vertex, double cost);

  /// Pop top vertex.
  gls::datastructures::Vertex popTopVertex();

  /// Get top vertex. Does not remove from the queue.
  gls::datastructures::Vertex getTopVertex();

  /// Get top vertex value.
  double getTopVertexValue();

  /// Remove vertex from search queue.
  /// \param[in] vertex Vertex to remove from the queue.
  /// \param[in] cost Cost associated with the vertex.
  void removeVertexWithValue(const gls::datastructures::Vertex vertex, double cost);

  /// Returns true if queue is empty.
  bool isEmpty();

  /// Returns the size of the queue.
  std::size_t getSize() const;

  /// Returns true if queue has vertex.
  /// \param[in] vertex Vertex to search for in the queue.
  bool hasVertexWithValue(const gls::datastructures::Vertex vertex, double cost);

  void printQueue() const;

private:
  /// Custom comparator used to order vertices.
  bool queueComparison(
      const std::pair<gls::datastructures::Vertex, double>&,
      const std::pair<gls::datastructures::Vertex, double>&) const;

  /// The underlying queue of vertices sorted by VertexQueueSortingFunction.
  VertexQueue mVertexQueue;

}; // SearchQueue

} // namespace datastructures
} // namespace gls

#endif // GLS_DATASTRUCTURES_SEARCHQUEUE_HPP_
