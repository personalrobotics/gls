/* Authors: Aditya Vamsikrishna Mandalika */

#include "gls/datastructures/SearchQueue.hpp"

#include <ompl/util/Console.h>  // OMPL_INFORM

namespace gls {
namespace datastructures {

using gls::datastructures::Vertex;

SearchQueue::SearchQueue(std::string name)
    : mVertexQueue([this](const double& lhs, const double& rhs) {
        return queueComparison(lhs, rhs);
      }),
      mName(name) {
  // Do nothing.
}

// ============================================================================
void SearchQueue::clear() { mVertexQueue.clear(); }

// ============================================================================
void SearchQueue::enqueueVertex(const Vertex& vertex, const double& cost) {
  SearchQueueIterator iterator =
      mVertexQueue.insert(std::pair<double, Vertex>(cost, vertex));
  // TODO(avk): Make the graph available to the search queue.
  // vertex.setSearchIterator(iterator);
}

// ============================================================================
Vertex SearchQueue::popTopVertex() {
  if (isEmpty()) {
    throw std::invalid_argument("Cannot pop from empty queue");
  }
  Vertex topVertex = mVertexQueue.begin()->second;
  dequeueVertex(topVertex);
  return topVertex;
}

// ============================================================================
Vertex SearchQueue::getTopVertex() const {
  if (isEmpty()) {
    throw std::invalid_argument("No top vertex in an empty queue");
  }
  return mVertexQueue.begin()->second;
}

// ============================================================================
double SearchQueue::getTopVertexCost() const {
  if (isEmpty()) {
    throw std::invalid_argument("No top vertex in an empty queue");
  }
  return mVertexQueue.begin()->first;
}

// ============================================================================
void SearchQueue::dequeueVertex(Vertex& vertex) {
  // If the queue is empty, no work to be done.
  if (this->isEmpty()) {
    return;
  }

  // TODO(avk): Make the graph available to the search queue.
  // If the vertex to dequeue is not in the queue, no work to be done.
  // if (!vertex.inSearchQueue()) {
  // return;
  // }
  // mVertexQueue.erase(vertex.getSearchIterator());

  // Clear the iterator and mark the vertex to not be in search queue.
  // vertex.clearSearchIterator();
}

// ============================================================================
bool SearchQueue::isEmpty() const { return mVertexQueue.empty(); }

// ============================================================================
std::string SearchQueue::getName() const { return mName; }

// ============================================================================
std::size_t SearchQueue::getSize() const { return mVertexQueue.size(); }

// ============================================================================
bool SearchQueue::queueComparison(const double& left,
                                  const double& right) const {
  if (left <= right) {
    return true;
  }
  return false;
}

// ============================================================================
void SearchQueue::printQueue() const {
  std::cout << "--------------------" << std::endl;
  std::cout << "Queue Size: " << mVertexQueue.size() << std::endl;
  std::cout << "--------------------" << std::endl;
  for (auto iterQ = mVertexQueue.begin(); iterQ != mVertexQueue.end();
       ++iterQ) {
    auto pair = *iterQ;
    std::cout << "Vertex: " << pair.first << " "
              << "Cost: " << pair.second << std::endl;
  }
  std::cout << "--------------------" << std::endl;
}

}  // namespace datastructures
}  // namespace gls
