/* Authors: Aditya Vamsikrishna Mandalika */

#include "gls/datastructures/SearchQueue.hpp"

#include <ompl/util/Console.h>

#include "gls/datastructures/Graph.hpp"

namespace gls {

GLS::SearchQueue::SearchQueue(std::string name)
    : mVertexQueue([this](const double& lhs, const double& rhs) {
        return queueComparison(lhs, rhs);
      }),
      mName(name) {
  // Do nothing.
}

// ============================================================================
void GLS::SearchQueue::setGraph(Graph* graph) { mGraph = graph; }

// ============================================================================
void GLS::SearchQueue::clear() { mVertexQueue.clear(); }

// ============================================================================
void GLS::SearchQueue::enqueueVertex(const Vertex& vertex, const double& cost) {
  SearchQueueIterator iterator =
      mVertexQueue.insert(std::pair<double, Vertex>(cost, vertex));

  auto& graph = *mGraph;
  graph[vertex].setSearchIterator(iterator);
}

// ============================================================================
Vertex GLS::SearchQueue::popTopVertex() {
  if (isEmpty()) {
    throw std::invalid_argument("Cannot pop from empty queue");
  }
  Vertex topVertex = mVertexQueue.begin()->second;
  dequeueVertex(topVertex);
  return topVertex;
}

// ============================================================================
Vertex GLS::SearchQueue::getTopVertex() const {
  if (isEmpty()) {
    throw std::invalid_argument("No top vertex in an empty queue");
  }
  return mVertexQueue.begin()->second;
}

// ============================================================================
double GLS::SearchQueue::getTopVertexCost() const {
  if (isEmpty()) {
    throw std::invalid_argument("No top vertex in an empty queue");
  }
  return mVertexQueue.begin()->first;
}

// ============================================================================
void GLS::SearchQueue::dequeueVertex(Vertex& vertex) {
  // If the queue is empty, no work to be done.
  if (this->isEmpty()) {
    return;
  }

  // If the vertex to dequeue is not in the queue, no work to be done.
  auto& graph = *mGraph;
  if (!graph[vertex].inSearchQueue()) {
    return;
  }
  mVertexQueue.erase(graph[vertex].getSearchIterator());

  // Clear the iterator and mark the vertex to not be in search queue.
  graph[vertex].clearSearchIterator();
}

// ============================================================================
bool GLS::SearchQueue::isEmpty() const { return mVertexQueue.empty(); }

// ============================================================================
std::string GLS::SearchQueue::getName() const { return mName; }

// ============================================================================
std::size_t GLS::SearchQueue::getSize() const { return mVertexQueue.size(); }

// ============================================================================
bool GLS::SearchQueue::queueComparison(const double& left,
                                       const double& right) const {
  if (left <= right) {
    return true;
  }
  return false;
}

// ============================================================================
void GLS::SearchQueue::printQueue() const {
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

}  // namespace gls
