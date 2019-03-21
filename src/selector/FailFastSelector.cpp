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
Path FailFastSelector::selectEdgeToEvaluate(Path path)
{
  auto graph = *mGraph;

  // Priority Function: g-value
  auto cmp = [&](const Edge& left, const Edge& right)
  {
    double estimateLeft = getPrior(left);
    double estimateRight = getPrior(right);

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
    boost::tie(uv, edgeExists) = edge(path[i], path[i - 1], graph);

    if (graph[uv].getEvaluationStatus() == EvaluationStatus::NotEvaluated)
    {
      qEdges.insert(uv);
    }
  }

  // There should always exist at least one unevaluated edge
  // when the selector is called.
  assert(qEdges.size() == 2);
  Edge eTop = *qEdges.begin();
  mGraph[eTop].setEvaluationStatus(EvaluationStatus::Evaluated);

  Path edgeToEvaluate;
  edgeToEvaluate.emplace_back(source(eTop, graph));
  edgeToEvaluate.emplace_back(target(eTop, graph));

  return edgeToEvaluate;
}

//==============================================================================
double FailFastSelector::getPrior(Edge edge)
{
  Vertex u = source(edge, graph);
  Vertex v = target(edge, graph);

  return mPriorMap[std::make_pair(u, v)];
}

} // namespace selector
} // namespace gls
