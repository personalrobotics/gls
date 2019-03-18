#include "gls/event/ConstantDepthEvent.hpp"

namespace gls {
namespace event {

using gls::datastructures::CollisionStatus;
using gls::datastructures::Edge;
using gls::datastructures::EvaluationStatus;
using gls::datastructures::Graph;
using gls::datastructures::Vertex;
using gls::datastructures::SearchQueue;

//==============================================================================
ConstantDepthEvent::ConstantDepthEvent(std::size_t depth)
: mDepthThreshold(depth)
{
  // Do nothing.
}

//==============================================================================
bool ConstantDepthEvent::isTriggered(const Vertex vertex)
{
  if (getDepth(vertex) == mDepthThreshold)
    return true;

  if (vertex == mTargetVertex)
    return true;

  return false;
}

//==============================================================================
void ConstantDepthEvent::updateVertexProperties(Vertex vertex)
{
  // Remove vertex if it already exists in the map.
  auto iterM = mVertexDepthMap.find(vertex);
  if (iterM != mVertexDepthMap.end())
    mVertexDepthMap.erase(iterM);

  // Add updated vertex.
  addVertexToMap(vertex);
}

//==============================================================================
void ConstantDepthEvent::updateVertexProperties(SearchQueue& updateQueue)
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
std::size_t ConstantDepthEvent::getDepth(Vertex vertex)
{
  auto iterM = mVertexDepthMap.find(vertex);
  assert(iterM == mVertexDepthMap.end());

  return mVertexDepthMap[vertex];
}

//==============================================================================
void ConstantDepthEvent::addVertexToMap(Vertex vertex)
{
  // Ensure the vertex does not already exist in the map.
  assert(mVertexDepthMap.find(vertex) == mVertexDepthMap.end());

  // Determine the parent depth.
  Vertex parent = mGraph[vertex].getParent();

  // Ensure the parent exists in the map.
  assert(mVertexDepthMap.find(parent) != mVertexDepthMap.end());

  // Get the edge and update the existenceProbability appropriately.
  Edge uv;
  bool edgeExists;
  boost::tie(uv, edgeExists) = edge(parent, vertex, mGraph);

  if (mGraph[uv].getEvaluationStatus() == EvaluationStatus::NotEvaluated)
  {
    // Increment depth by 1 over the parent's depth and add to map.
    mVertexDepthMap.emplace(vertex, mVertexDepthMap[parent] + 1);
  }
  else
  {
    // Edge should not have been considered if it was evaluated in collision.
    assert(mGraph[uv].getCollisionStatus() == CollisionStatus::Free);

    // Same depth as parent if the edge to vertex was evaluated.
    mVertexDepthMap.emplace(vertex, mVertexDepthMap[parent]);
  }
}

} // namespace event
} // namespace gls
