#include "gls/selector/AlternateSelector.hpp"

#include "gls/selector/BackwardSelector.hpp"
#include "gls/selector/ForwardSelector.hpp"

namespace gls {
namespace selector {

using gls::datastructures::Edge;
using gls::datastructures::Path;

//==============================================================================
AlternateSelector::AlternateSelector() {
  // Setup forward selector.
  mForwardSelector = std::make_shared<gls::selector::ForwardSelector>();
  mForwardSelector->setup(mGraph);

  // Setup backward selector.
  mBackwardSelector = std::make_shared<gls::selector::BackwardSelector>();
  mBackwardSelector->setup(mGraph);
}

//==============================================================================
Edge AlternateSelector::selectEdgeToEvaluate(Path path) {
  // Flip the boolean.
  mUseForwardSelector = !mUseForwardSelector;

  if (mUseForwardSelector)
    return mForwardSelector->selectEdgeToEvaluate(path);
  else
    return mBackwardSelector->selectEdgeToEvaluate(path);
}

}  // namespace selector
}  // namespace gls
