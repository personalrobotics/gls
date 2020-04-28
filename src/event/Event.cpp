/* Authors: Aditya Vamsikrishna Mandalika */

#include "gls/event/Event.hpp"

#include "gls/datastructures/Graph.hpp"
#include "gls/datastructures/SearchQueue.hpp"

namespace gls {

//==============================================================================
GLS::Event::Event() {
  // Do nothing.
}

//==============================================================================
void GLS::Event::setup(Graph* graph, Vertex& source, Vertex& target) {
  mGraph = graph;
  mSourceVertex = source;
  mTargetVertex = target;
}

//==============================================================================
void GLS::Event::updateVertexProperties(SearchQueuePtr updateQueue) {
  // Access the graph.
  auto graph = *mGraph;

  while (!updateQueue->isEmpty()) {
    // Update the top vertex.
    Vertex vertex = updateQueue->popTopVertex();
    updateVertexProperties(vertex);

    auto children = graph[vertex].getChildren();
    for (auto iterV = children.begin(); iterV != children.end(); ++iterV) {
      // Add the children into the queue for update.
      assert(
          !updateQueue->enqueueVertex(*iterV, graph[*iterV].getCostToCome()));
      updateQueue->enqueueVertex(*iterV, graph[*iterV].getCostToCome());
    }
  }
}
}  // namespace gls
