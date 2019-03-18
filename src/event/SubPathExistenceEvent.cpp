#include "gls/event/SubPathExistenceEvent.hpp"

namespace gls {
namespace event {

using gls::datastructures::CollisionStatus;
using gls::datastructures::Edge;
using gls::datastructures::EvaluationStatus;
using gls::datastructures::Graph;
using gls::datastructures::Vertex;
using gls::datastructures::SearchQueue;

//==============================================================================
SubPathExistenceEvent::SubPathExistenceEvent(edgeToPriorMap& priorMap, double existenceThreshold)
: mPriorMap(priorMap)
, mExistenceThreshold(existenceThreshold)
{
  // Do nothing.
}

//==============================================================================
bool SubPathExistenceEvent::isTriggered(const Vertex vertex)
{
  if (getExistenceThreshold(vertex) <= mExistenceThreshold)
    return true;

  if (vertex == mTargetVertex)
    return true;

  return false;
}

//==============================================================================
void SubPathExistenceEvent::updateVertexProperties(Vertex vertex)
{
  // Remove vertex if it already exists in the map.
  auto iterM = mSubPathExistenceMap.find(vertex);
  if (iterM != mSubPathExistenceMap.end())
    mSubPathExistenceMap.erase(iterM);

  // Add updated vertex.
  addVertexToMap(vertex);
}

//==============================================================================
void SubPathExistenceEvent::updateVertexProperties(SearchQueue& updateQueue)
{
	while (!updateQueue.isEmpty())
	{
		// Update the top vertex.
		Vertex vertex = updateQueue.popTopVertex();	
		updateVertexProperties(vertex);

		auto children = mGraph[vertex].getChildren();
	  for (auto iterV = children.begin(); iterV != children.end(); ++iterV)
	  {
	  	// Add the children into the queue for update.
	    assert(!updateQueue.hasVertexWithValue(*iterV, mGraph[*iterV].getCostToCome()));
    	updateQueue.addVertexWithValue(*iterV, mGraph[*iterV].getCostToCome());
	  }
	}
}

//==============================================================================
double SubPathExistenceEvent::getExistenceProbability(Vertex vertex)
{
  auto iterM = mSubPathExistenceMap.find(vertex);
  assert(iterM == mSubPathExistenceMap.end());

  return mSubPathExistenceMap[vertex];
}

//==============================================================================
void SubPathExistenceEvent::addVertexToMap(Vertex vertex)
{
  // Ensure the vertex does not already exist in the map.
  assert(mSubPathExistenceMap.find(vertex) == mSubPathExistenceMap.end());

  // Determine the parent probability.
  Vertex parent = mGraph[vertex].getParent();

  // Ensure the parent exists in the map.
  assert(mSubPathExistenceMap.find(parent) != mSubPathExistenceMap.end());

  // Get the edge and update the existenceProbability appropriately.
  Edge uv;
  bool edgeExists;
  boost::tie(uv, edgeExists) = edge(parent, vertex, mGraph);

  if (mGraph[uv].getEvaluationStatus() == EvaluationStatus::NotEvaluated)
  {
    mSubPathExistenceMap.emplace(vertex, mSubPathExistenceMap[parent]*getPrior(uv));
  }
  else
  {
    assert(mGraph[uv].getCollisionStatus() == CollisionStatus::Free);
    mSubPathExistenceMap.emplace(vertex, mSubPathExistenceMap[parent]);
  }
}

//==============================================================================
double SubPathExistenceEvent::getPrior(Edge edge)
{
  Vertex u = source(edge, mGraph);
  Vertex v = target(edge, mGraph);

  return mPriorMap[std::make_pair(u, v)];
}

} // namespace event
} // namespace gls
