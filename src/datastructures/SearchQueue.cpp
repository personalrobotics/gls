/* Authors: Aditya Vamsikrishna Mandalika */

#include "gls/datastructures/SearchQueue.hpp"

#include <ompl/util/Console.h>  // OMPL_INFORM

namespace gls {
namespace datastructures {

using gls::datastructures::Vertex;

SearchQueue::SearchQueue()
    : mVertexQueue(
          [this](const std::pair<gls::datastructures::Vertex, double>& lhs,
                 const std::pair<gls::datastructures::Vertex, double>& rhs) {
            return queueComparison(lhs, rhs);
          }) {
  // Do Nothing.
}

// ============================================================================
void SearchQueue::clear() { mVertexQueue.clear(); }

// ============================================================================
void SearchQueue::addVertexWithValue(Vertex vertex, double cost) {
  std::pair<Vertex, double> addPair = std::make_pair(vertex, cost);
  mVertexQueue.insert(addPair);
}

// ============================================================================
Vertex SearchQueue::popTopVertex() {
  Vertex topVertex = (*mVertexQueue.begin()).first;
  mVertexQueue.erase(mVertexQueue.begin());

  return topVertex;
}

// ============================================================================
Vertex SearchQueue::getTopVertex() { return (*mVertexQueue.begin()).first; }

// ============================================================================
double SearchQueue::getTopVertexValue() {
  return (*mVertexQueue.begin()).second;
}

// ============================================================================
void SearchQueue::removeVertexWithValue(Vertex vertex, double cost) {
  auto iterQ = mVertexQueue.find(std::make_pair(vertex, cost));
  if (iterQ != mVertexQueue.end()) mVertexQueue.erase(iterQ);
}

// ============================================================================
bool SearchQueue::isEmpty() {
  if (mVertexQueue.empty()) return true;

  return false;
}

// ============================================================================
std::size_t SearchQueue::getSize() const { return mVertexQueue.size(); }

// ============================================================================
bool SearchQueue::hasVertexWithValue(const Vertex vertex, double cost) {
  auto iterQ = mVertexQueue.find(std::make_pair(vertex, cost));
  if (iterQ != mVertexQueue.end()) return true;

  return false;
}

// ============================================================================
bool SearchQueue::queueComparison(
    const std::pair<gls::datastructures::Vertex, double>& left,
    const std::pair<gls::datastructures::Vertex, double>& right) const {
  if (left.second < right.second)
    return true;
  else if (left.second > right.second)
    return false;
  else {
    return left.first < right.first;
  }
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
