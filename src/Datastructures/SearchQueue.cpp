/* Authors: Aditya Vamsikrishna Mandalika */

#include "GLS/Datastructures/SearchQueue.hpp"

#include <ompl/util/Console.h> // OMPL_INFORM

namespace gls {
namespace datastructures {

using gls::datastructures::Vertex;

SearchQueue::SearchQueue()
  : mVertexQueue(
        [this](
            const std::pair<gls::datastructures::Vertex, double>& lhs,
            const std::pair<gls::datastructures::Vertex, double>& rhs) {
          return queueComparison(lhs, rhs);
        })
{
  // Do Nothing.
}

// ============================================================================
void SearchQueue::clear()
{
  mVertexQueue.clear();
}

// ============================================================================
void SearchQueue::addVertexWithValue(Vertex vertex, double cost)
{
  std::pair<Vertex, double> addPair = std::make_pair(vertex, cost);
  mVertexQueue.insert(addPair);
}

// ============================================================================
Vertex SearchQueue::popTopVertex()
{
  Vertex topVertex = (*mVertexQueue.begin()).first;
  mVertexQueue.erase(mVertexQueue.begin());

  return topVertex;
}

// ============================================================================
Vertex SearchQueue::getTopVertex()
{
  return (*mVertexQueue.begin()).first;
}

// ============================================================================
double SearchQueue::getTopVertexValue()
{
  return (*mVertexQueue.begin()).second;
}

// ============================================================================
void SearchQueue::removeVertexWithValue(Vertex vertex, double cost)
{
  auto iterQ = mVertexQueue.find(std::make_pair(vertex, cost));
  if (iterQ != mVertexQueue.end())
    mVertexQueue.erase(iterQ);
}

// ============================================================================
bool SearchQueue::isEmpty()
{
  if (mVertexQueue.empty())
    return true;

  return false;
}

// ============================================================================
bool SearchQueue::hasVertexWithValue(const Vertex vertex, double cost)
{
  auto iterQ = mVertexQueue.find(std::make_pair(vertex, cost));
  if (iterQ != mVertexQueue.end())
    return true;

  return false;
}

// ============================================================================
bool queueComparison(
    const std::pair<gls::datastructures::Vertex, double>& left,
    const std::pair<gls::datastructures::Vertex, double>& right)
{
  if (left.second < right.second)
    return true;
  else
    return false;
}

} // datastructures
} // gls
