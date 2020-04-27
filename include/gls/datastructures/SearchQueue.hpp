/* Author: Aditya Vamsikrishna Mandalika */

#ifndef GLS_DATASTRUCTURES_SEARCHQUEUE_HPP_
#define GLS_DATASTRUCTURES_SEARCHQUEUE_HPP_

#include <ompl/base/Cost.h>
#include <ompl/datastructures/BinaryHeap.h>

#include <functional>
#include <map>

#include "gls/datastructures/Types.hpp"

namespace gls {
namespace datastructures {

/// \brief A queue consists of vertices to be expanded during search. The queue
/// is implemented as a ordered list of vertices prioritized by its total cost,
/// specifically a multimap.
class SearchQueue {
 public:
  /// \brief The function signature of the sorting function for the queue.
  using SearchQueueComparator =
      std::function<bool(const double&, const double&)>;

  /// \brief The underlying vertex queue implemented as a multimap.
  using VertexQueueMMap =
      std::multimap<double, gls::datastructures::Vertex, SearchQueueComparator>;

  /// \brief An iterator into the multimap.
  /// Map has the important property that inserting a new element into a map
  /// does not invalidate iterators that point to existing elements. Erasing an
  /// element from a map also does not invalidate any iterators, except, of
  /// course, iterators that actually point to the element that is being erased.
  using SearchQueueIterator = VertexQueueMMap::iterator;

  /// \brief Constructor.
  explicit SearchQueue(std::string name = "SearchQueue");

  /// \brief Destructor.
  virtual ~SearchQueue() = default;

  /// \brief Clear the search queue.
  void clear();

  /// Add vertex to the search queue.
  /// \param[in] vertex Vertex to add to the search queue.
  void enqueueVertex(const gls::datastructures::Vertex& vertex,
                     const double& cost);

  /// Pop top vertex.
  gls::datastructures::Vertex popTopVertex();

  /// Get top vertex. Does not remove from the queue.
  gls::datastructures::Vertex getTopVertex() const;

  /// Get top vertex value.
  double getTopVertexCost() const;

  /// Remove vertex from search queue.
  /// \param[in] vertex Vertex to remove from the queue.
  void dequeueVertex(gls::datastructures::Vertex& vertex);

  /// Returns true if queue is empty.
  bool isEmpty() const;

  /// Returns the name of the queue.
  std::string getName() const;

  /// Returns the size of the queue.
  std::size_t getSize() const;

  /// Prints all the vertices in the queue as list of [ID :: Cost].
  void printQueue() const;

 private:
  /// \brief A custom comparator function for the vertices in the queue.
  bool queueComparison(const double&, const double&) const;

  /// \brief Name of the search queue.
  std::string mName;

  /// \brief THe underlying queue of vertices.
  VertexQueueMMap mVertexQueue;

};  // SearchQueue

}  // namespace datastructures
}  // namespace gls

#endif  // GLS_DATASTRUCTURES_SEARCHQUEUE_HPP_
