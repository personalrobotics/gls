#include "gls/selector/AlternateSelector.hpp"
#include "gls/selector/BackwardSelector.hpp"
#include "gls/selector/ForwardSelector.hpp"

namespace gls {
namespace selector {

using gls::datastructures::Path;
using gls::datastructures::Vertex;

//==============================================================================
AlternateSelector::AlternateSelector()
{
  // Setup forward selector.
  mForwardSelector = std::make_shared<gls::selector::ForwardSelector>();
  mForwardSelector->setup(mGraph, mSourceVertex, mTargetVertex);

  // Setup backward selector.
  mBackwardSelector = std::make_shared<gls::selector::BackwardSelector>();
  mBackwardSelector->setup(mGraph, mSourceVertex, mTargetVertex);
}

//==============================================================================
Path AlternateSelector::selectEdgesToEvaluate(gls::datastructures::Path path)
{
  // Flip the boolean.
  mUseForwardSelector = !mUseForwardSelector;

  if (mUseForwardSelector)
    return mForwardSelector->selectEdgesToEvaluate(path);
  else
    return mBackwardSelector->selectEdgesToEvaluate(path);
}

} // namespace selector
} // namespace gls
