#include "gls/selector/BackwardSelector.hpp"

#include <algorithm> // std::reverse

namespace gls {
namespace selector {

using gls::datastructures::Edge;
using gls::datastructures::EvaluationStatus;
using gls::datastructures::Graph;
using gls::datastructures::Path;
using gls::datastructures::Vertex;

//==============================================================================
BackwardSelector::BackwardSelector()
{
  // Do nothing.
}

//==============================================================================
Path BackwardSelector::selectEdgesToEvaluate(gls::datastructures::Path path)
{
  // If the path is to the target, evaluate the entire path.
  if (path[0] == mTargetVertex)
    return path;

  // Else return the first unevaluated edge closest to target.
  Path edgesToEvaluate;
  for (std::size_t i = 0; i < path.size() - 1; ++i)
  {
    Edge uv;
    bool edgeExists;
    boost::tie(uv, edgeExists) = edge(path[i + 1], path[i], mGraph);

    if (mGraph[uv].getEvaluationStatus() == EvaluationStatus::NotEvaluated)
    {
      edgesToEvaluate.emplace_back(path[i + 1]);
      edgesToEvaluate.emplace_back(path[i]);
      break;
    }
  }
  return edgesToEvaluate;
}

//==============================================================================
Path BackwardSelector::rankEdgesByUtilityInEvaluation(
    gls::datastructures::Path& path)
{
  // Do nothing.
  return path;
}

} // namespace selector
} // namespace gls
