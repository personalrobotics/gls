#include "gls/selector/FailFastSelector.hpp"

namespace gls {
namespace selector {

using gls::datastructures::Edge;
using gls::datastructures::EvaluationStatus;
using gls::datastructures::Path;

//==============================================================================
FailFastSelector::FailFastSelector(const edgeToPriorMap& priorMap)
: mPriorMap(priorMap)
{
  // Do nothing.
}

//==============================================================================
Path FailFastSelector::selectEdgesToEvaluate(Path path)
{
  return path;
}

} // namespace selector
} // namespace gls
