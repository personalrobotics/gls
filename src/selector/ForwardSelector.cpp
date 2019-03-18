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
Edge ForwardSelector::selectEdgeToEvaluate(Path path)
{
  // Return the first unevaluated edge closest to source.
  for (std::size_t i = path.size() - 1; i > 0; --i)
  {
    std::cout << "Edge " << path[i] << " " << path[i-1] << std::endl;
    Edge uv;
    bool edgeExists;
    boost::tie(uv, edgeExists) = edge(path[i], path[i - 1], mGraph);

    if (mGraph[uv].getEvaluationStatus() == EvaluationStatus::NotEvaluated)
      return uv;
  }
}

} // namespace selector
} // namespace gls
