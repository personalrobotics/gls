#include "gls/selector/ForwardSelector.hpp"

namespace gls {
namespace selector {

using gls::datastructures::Edge;
using gls::datastructures::EvaluationStatus;
using gls::datastructures::Graph;
using gls::datastructures::Path;
using gls::datastructures::Vertex;

//==============================================================================
ForwardSelector::ForwardSelector()
{
  // Do nothing.
}

//==============================================================================
Path ForwardSelector::selectEdgesToEvaluate(gls::datastructures::Path path)
{
  // If the path is to the target, evaluate the entire path.
  if (path[path.size() - 1] == mTargetVertex)
    return path;

  // Else return the first unevaluated edge closest to source.
  Path edgesToEvaluate;
  for (std::size_t i = 0; i < path.size() - 1; ++i)
  {
    Edge uv;
    bool edgeExists;
    boost::tie(uv, edgeExists) = edge(path[i], path[i + 1], mGraph);

    if (mGraph[uv].getEvaluationStatus() == EvaluationStatus::NotEvaluated)
    {
      edgesToEvaluate.emplace_back(path[i]);
      edgesToEvaluate.emplace_back(path[i + 1]);
      break;
    }
  }
  return edgesToEvaluate;
}

//==============================================================================
Path ForwardSelector::rankEdgesByUtilityInEvaluation(
    gls::datastructures::Path& path)
{
  return path;
  // Do nothing.
}

} // namespace selector
} // namespace gls
