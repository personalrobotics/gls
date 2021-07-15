#include "gls/event/ConstantDepthEvent.hpp"

namespace gls {
namespace event {

using gls::datastructures::CollisionStatus;
using gls::datastructures::Edge;
using gls::datastructures::EvaluationStatus;
using gls::datastructures::Graph;
using gls::datastructures::SearchQueue;
using gls::datastructures::Vertex;

//==============================================================================
ConstantDepthEvent::ConstantDepthEvent(std::size_t depth) : mDepthThreshold(depth) {
  // Do nothing.
}

//==============================================================================
bool ConstantDepthEvent::isTriggered(const Vertex& vertex) {
  if(std::find(mTargetVertices.begin(), mTargetVertices.end(), vertex) != mTargetVertices.end()) {
    return true;
  }

  if (getDepth(vertex) == mDepthThreshold) {
    return true;
  }
  return false;
}

//==============================================================================
void ConstantDepthEvent::updateVertexProperties(Vertex& vertex) {
  // Remove vertex if it already exists in the map.
  auto iterM = mVertexDepthMap.find(vertex);
  if (iterM != mVertexDepthMap.end())
    mVertexDepthMap.erase(iterM);

  // Add updated vertex.
  updateVertexInMap(vertex);
}

//==============================================================================
std::size_t ConstantDepthEvent::getDepth(const Vertex& vertex) {
  auto iterM = mVertexDepthMap.find(vertex);
  assert(iterM != mVertexDepthMap.end());

  return mVertexDepthMap[vertex];
}

//==============================================================================
void ConstantDepthEvent::updateVertexInMap(Vertex& vertex) {
  // Access the graph.
  auto graph = *mGraph;

  // Ensure the vertex does not already exist in the map.
  assert(mVertexDepthMap.find(vertex) == mVertexDepthMap.end());

  // Add the vertex if it is the source vertex.
  if (vertex == mSourceVertex) {
    // Increment depth by 1 over the parent's depth and add to map.
    mVertexDepthMap.emplace(vertex, 0);
    return;
  }

  // Determine the parent depth.
  Vertex parent = graph[vertex].getParent();

  // If the parent is same as vertex, remove the vertex from the map.
  // This is not really required but we do it to decrease the map size.
  if (parent == vertex) {
    auto iterM = mVertexDepthMap.find(vertex);
    if (iterM != mVertexDepthMap.end())
      mVertexDepthMap.erase(iterM);
    return;
  }

  // Ensure the parent exists in the map.
  assert(mVertexDepthMap.find(parent) != mVertexDepthMap.end());

  // Get the edge and update the depth appropriately.
  Edge uv;
  bool edgeExists;
  boost::tie(uv, edgeExists) = edge(parent, vertex, graph);

  // Edge should not have been considered if it was evaluated in collision.
  assert(graph[uv].getCollisionStatus() == CollisionStatus::Free);
  if (graph[uv].getEvaluationStatus() == EvaluationStatus::NotEvaluated) {
    // Increment depth by 1 over the parent's depth and add to map.
    mVertexDepthMap.emplace(vertex, mVertexDepthMap[parent] + 1);
  } else {
    // Same depth as parent if the edge to vertex was evaluated.
    mVertexDepthMap.emplace(vertex, mVertexDepthMap[parent]);
  }
}

} // namespace event
} // namespace gls
