#include "gls/selector/BackwardSelector.hpp"

#include "gls/datastructures/Graph.hpp"

namespace gls {

//==============================================================================
BackwardSelector::BackwardSelector() {
  // Do nothing.
}

//==============================================================================
Edge BackwardSelector::selectEdgeToEvaluate(Path path) {
  // Access the graph.
  auto graph = *mGraph;
  Edge edgeToEvaluate;

  // Return the first unevaluated edge closest to target.
  for (std::size_t i = 0; i < path.size() - 1; ++i) {
    bool edgeExists;
    boost::tie(edgeToEvaluate, edgeExists) = edge(path[i + 1], path[i], graph);

    if (graph[edgeToEvaluate].getEvaluationStatus() ==
        EvaluationStatus::NotEvaluated)
      break;
  }

  return edgeToEvaluate;
}

}  // namespace gls
