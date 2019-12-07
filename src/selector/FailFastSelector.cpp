#include "gls/selector/FailFastSelector.hpp"

namespace gls {
namespace selector {

using gls::datastructures::Edge;
using gls::datastructures::EvaluationStatus;
using gls::datastructures::Path;
using gls::datastructures::Vertex;

//==============================================================================
FailFastSelector::FailFastSelector(edgeToPriorMap& priorMap)
  : mPriorMap(priorMap) {
  // Do nothing.
}

//==============================================================================
Edge FailFastSelector::selectEdgeToEvaluate(Path path) {
  auto graph = *mGraph;
  Edge edgeToEvaluate;

  double minPrior = 1.0;

  // Return the first unevaluated edge closest to source.
  for (std::size_t i = path.size() - 1; i > 0; --i) {
    Edge uv;
    bool edgeExists;
    boost::tie(uv, edgeExists) = edge(path[i], path[i - 1], graph);

    if (graph[uv].getEvaluationStatus() == EvaluationStatus::NotEvaluated) {
      if (getPrior(uv) <= minPrior) {
        minPrior = getPrior(uv);
        edgeToEvaluate = uv;
      }
    }
  }

  return edgeToEvaluate;
}

//==============================================================================
double FailFastSelector::getPrior(Edge edge) {
  auto graph = *mGraph;

  Vertex u = source(edge, graph);
  Vertex v = target(edge, graph);

  return mPriorMap[std::make_pair(u, v)];
}

} // namespace selector
} // namespace gls
