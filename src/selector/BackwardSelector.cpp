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
Edge BackwardSelector::selectEdgeToEvaluate(Path path)
{
  // Return the first unevaluated edge closest to target.
  for (std::size_t i = 0; i < path.size() - 1; ++i)
  {
    Edge uv;
    bool edgeExists;
    boost::tie(uv, edgeExists) = edge(path[i + 1], path[i], mGraph);

    if (mGraph[uv].getEvaluationStatus() == EvaluationStatus::NotEvaluated)
      return uv;
  }
}

} // namespace selector
} // namespace gls
