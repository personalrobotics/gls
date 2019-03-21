#include "gls/selector/BackwardSelector.hpp"

namespace gls {
namespace selector {

using gls::datastructures::Edge;
using gls::datastructures::EvaluationStatus;
using gls::datastructures::Path;

//==============================================================================
BackwardSelector::BackwardSelector()
{
  // Do nothing.
}

//==============================================================================
Path BackwardSelector::selectEdgeToEvaluate(Path path)
{
  // Access the graph.
  auto graph = *mGraph;

  Path edgeToEvaluate;

  // Return the first unevaluated edge closest to target.
  for (std::size_t i = 0; i < path.size() - 1; ++i)
  {
    Edge uv;
    bool edgeExists;
    boost::tie(uv, edgeExists) = edge(path[i + 1], path[i], graph);

    if (graph[uv].getEvaluationStatus() == EvaluationStatus::NotEvaluated)
    {
      edgeToEvaluate.emplace_back(path[i + 1]);
      edgeToEvaluate.emplace_back(path[i]);
      break;
    }
  }

  // There should always exist at least one unevaluated edge
  // when the selector is called.
  assert(edgeToEvaluate.size() == 2);
  return edgeToEvaluate;
}

} // namespace selector
} // namespace gls
