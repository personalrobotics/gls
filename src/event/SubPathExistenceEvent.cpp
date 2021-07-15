#include "gls/event/SubPathExistenceEvent.hpp"

namespace gls {
namespace event {

using gls::datastructures::CollisionStatus;
using gls::datastructures::Edge;
using gls::datastructures::EvaluationStatus;
using gls::datastructures::Graph;
using gls::datastructures::SearchQueue;
using gls::datastructures::Vertex;

//==============================================================================
SubPathExistenceEvent::SubPathExistenceEvent(edgeToPriorMap& priorMap, double existenceThreshold)
  : mPriorMap(priorMap), mExistenceThreshold(existenceThreshold) {
  // Do nothing.
}

//==============================================================================
bool SubPathExistenceEvent::isTriggered(const Vertex& vertex) {
  if(std::find(mTargetVertices.begin(), mTargetVertices.end(), vertex) != mTargetVertices.end()) {
    return true;
  }

  if (getExistenceThreshold(vertex) <= mExistenceThreshold) {
    return true;
  }

  return false;
}

//==============================================================================
void SubPathExistenceEvent::updateVertexProperties(Vertex& vertex) {
  // Remove vertex if it already exists in the map.
  auto iterM = mSubPathExistenceMap.find(vertex);
  if (iterM != mSubPathExistenceMap.end()) {
    mSubPathExistenceMap.erase(iterM);
  }

  // Add updated vertex.
  updateVertexInMap(vertex);
}

//==============================================================================
double SubPathExistenceEvent::getExistenceProbability(const Vertex& vertex) {
  auto iterM = mSubPathExistenceMap.find(vertex);
  assert(iterM == mSubPathExistenceMap.end());

  return mSubPathExistenceMap[vertex];
}

//==============================================================================
void SubPathExistenceEvent::updateVertexInMap(Vertex& vertex) {
  // Access the graph.
  auto graph = *mGraph;

  // Ensure the vertex does not already exist in the map.
  assert(mSubPathExistenceMap.find(vertex) == mSubPathExistenceMap.end());

  // Add the vertex it is is source vertex.
  if (vertex == mSourceVertex) {
    // Assign a prior of 1 by default.
    mSubPathExistenceMap.emplace(vertex, 1.0);
    return;
  }

  // Determine the parent probability.
  Vertex parent = graph[vertex].getParent();

  // If the parent is same as vertex, remove the vertex from the map.
  // This is not really required but we do it to decrease the map size.
  if (parent == vertex) {
    auto iterM = mSubPathExistenceMap.find(vertex);
    if (iterM != mSubPathExistenceMap.end())
      mSubPathExistenceMap.erase(iterM);
    return;
  }

  // Ensure the parent exists in the map.
  assert(mSubPathExistenceMap.find(parent) != mSubPathExistenceMap.end());

  // Get the edge and update the existenceProbability appropriately.
  Edge uv;
  bool edgeExists;
  boost::tie(uv, edgeExists) = edge(parent, vertex, graph);

  // Edge should not have been considered if it was evaluated in collision.
  assert(graph[uv].getCollisionStatus() == CollisionStatus::Free);
  if (graph[uv].getEvaluationStatus() == EvaluationStatus::NotEvaluated) {
    // Update the prior if the edge from parent has not been evaluated yet.
    mSubPathExistenceMap.emplace(vertex, mSubPathExistenceMap[parent] * getPrior(uv));
  } else {
    // Same probability of existence if edge from parent has been evaluated.
    mSubPathExistenceMap.emplace(vertex, mSubPathExistenceMap[parent]);
  }
}

//==============================================================================
double SubPathExistenceEvent::getPrior(const Edge& edge) {
  Vertex u = source(edge, graph);
  Vertex v = target(edge, graph);

  return mPriorMap[std::make_pair(u, v)];
}

} // namespace event
} // namespace gls
