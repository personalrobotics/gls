#include "gls/event/ConstantDepthEvent.hpp"

#include <iostream>  // std::invalid_argument

namespace gls {
namespace event {

using gls::datastructures::Graph;
using gls::datastructures::Vertex;
using gls::datastructures::SearchQueue;

//==============================================================================
ConstantDepthEvent::ConstantDepthEvent()
{
  // Do nothing.
}

//==============================================================================
bool ConstantDepthEvent::isTriggered(const Vertex vertex) const
{
	double depth = getDepth(vertex);

  if (depth == mDepthThreshold)
  	return true;

  return false;
}

//==============================================================================
void ConstantDepthEvent::updateVertexProperties(
    Vertex vertex, vertexUpdateOption cascade)
{
	// Remove vertex if it already exists in the map.
	auto iterM = mVertexDepthMap.find(vertex);
	if (iterM != mVertexDepthMap.end())
		mVertexDepthMap.erase(iterM);

	// Add updated vertex.
	addVertexToMap(vertex);
}

//==============================================================================
void ConstantDepthEvent::updateVertexProperties(SearchQueue searchQueue)
{
	Vertex vertex = searchQueue.popTopVertex();
	updateVertexProperties(vertex);

	auto children = mGraph[vertex].getChildren();
	for (auto iterV = children.begin(); iterV != children.end(); ++iterV)
	{
		if (searchQueue.hasVertexWithValue(*iterV, mGraph[*iterV].getCostToCome()))
			continue;
		
		updateVertexProperties(*iterV);
	}
}

//==============================================================================
double ConstantDepthEvent::getDepth(Vertex vertex)
{
	auto iterM = mVertexDepthMap.find(vertex);

	if (iterM == mVertexDepthMap.end())
		throw std::invalid_argument("Vertex has not been registered.")

	return mVertexDepthMap(*iterM);
}

//==============================================================================
void ConstantDepthEvent::addVertexToMap(Vertex vertex)
{
	// Ensure the vertex does not already exist in the map.
  assert(mVertexDepthMap.find(vertex) == mVertexDepthMap.end());

  // Determine the parent depth.
  auto iterM = mVertexDepthMap.find(mGraph[vertex].getParent());
  assert(iterM != mVertexDepthMap.end());

  // Increment depth by 1 over the parent's depth and add to map.
  mVertexDepthMap.emplace(vertex, mVertexDepthMap(*iterM) + 1);
}

} // namespace event
} // namespace gls
