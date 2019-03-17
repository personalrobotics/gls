#include "gls/selector/FailFastSelector.hpp"

namespace gls {
namespace selector {

using gls::datastructures::Edge;
using gls::datastructures::EvaluationStatus;
using gls::datastructures::Path;
using gls::datastructures::Vertex;

//==============================================================================
FailFastSelector::FailFastSelector(edgeToPriorMap& priorMap)
: mPriorMap(priorMap)
{
  // Do nothing.
}

//==============================================================================
Edge FailFastSelector::selectEdgeToEvaluate(Path path)
{
  // Priority Function: g-value
  auto cmp = [&](const Edge& left, const Edge& right)
  {
    double estimateLeft = evaluatePrior(left);
    double estimateRight = evaluatePrior(right);

    if (estimateRight > estimateLeft)
      return true;
    if (estimateLeft > estimateRight)
      return false;
    if (left < right)
      return true;
    else
      return false;
  };

  std::set<Edge, decltype(cmp)> qEdges(cmp);

  // Return the first unevaluated edge closest to source.
  for (std::size_t i = path.size() - 1; i > 0; --i)
  {
    Edge uv;
    bool edgeExists;
    boost::tie(uv, edgeExists) = edge(path[i], path[i - 1], mGraph);

    if (mGraph[uv].getEvaluationStatus() == EvaluationStatus::NotEvaluated)
    {
      qEdges.insert(uv);
    }
  }
}

//==============================================================================
double FailFastSelector::evaluatePrior(Edge edge)
{
  Vertex u = source(edge, mGraph);
  Vertex v = target(edge, mGraph);

  return mPriorMap[std::make_pair(u, v)];
}

} // namespace selector
} // namespace gls
