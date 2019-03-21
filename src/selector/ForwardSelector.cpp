#include "gls/selector/ForwardSelector.hpp"

namespace gls {
namespace selector {

using gls::datastructures::Edge;
using gls::datastructures::EvaluationStatus;
using gls::datastructures::Path;

//==============================================================================
ForwardSelector::ForwardSelector()
{
  // Do nothing.
}

//==============================================================================
Path ForwardSelector::selectEdgeToEvaluate(Path path)
{
  // Access the graph.
  auto graph = *mGraph;

  Path edgeToEvaluate;

  // Return the first unevaluated edge closest to source.
  for (std::size_t i = path.size() - 1; i > 0; --i)
  {
    Edge uv;
    bool edgeExists;
    boost::tie(uv, edgeExists) = edge(path[i], path[i - 1], graph);

    if (graph[uv].getEvaluationStatus() == EvaluationStatus::NotEvaluated)
    {
      edgeToEvaluate.emplace_back(path[i]);
      edgeToEvaluate.emplace_back(path[i - 1]);
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
