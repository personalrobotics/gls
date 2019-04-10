/* Authors: Aditya Vamsikrishna Mandalika */

#include "gls/event/Event.hpp"

namespace gls {
namespace event {

using gls::datastructures::Graph;
using gls::datastructures::Vertex;
using gls::datastructures::SearchQueue;

//==============================================================================
Event::Event()
{
  // Do nothing.
}

//==============================================================================
void Event::setup(Graph* graph, Vertex source, Vertex target)
{
  mGraph = graph;
  mSourceVertex = source;
  mTargetVertex = target;
}

//==============================================================================
void Event::updateVertexProperties(SearchQueue& updateQueue)
{
  // Access the graph.
  auto graph = *mGraph;

	while (!updateQueue.isEmpty())
	{
		// Update the top vertex.
		Vertex vertex = updateQueue.popTopVertex();	
		updateVertexProperties(vertex);

		auto children = graph[vertex].getChildren();
	  for (auto iterV = children.begin(); iterV != children.end(); ++iterV)
	  {
	  	// Add the children into the queue for update.
	    assert(!updateQueue.hasVertexWithValue(*iterV, graph[*iterV].getCostToCome()));
    	updateQueue.addVertexWithValue(*iterV, graph[*iterV].getCostToCome());
	  }
	}
}

} // event
} // gls
