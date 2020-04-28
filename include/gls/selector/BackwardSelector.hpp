#ifndef GLS_SELECTOR_BACKWARDSELECTOR_HPP_
#define GLS_SELECTOR_BACKWARDSELECTOR_HPP_

#include "gls/selector/Selector.hpp"

namespace gls {

/// Selector that evaluates the edge on the path closest to the target.
class BackwardSelector : public GLS::Selector {
 public:
  /// Constructor.
  BackwardSelector();

  /// Documentation inherited.
  Edge selectEdgeToEvaluate(Path path) override;
};

}  // namespace gls

#endif  // GLS_SELECTOR_BACKWARDSELECTOR_HPP_
