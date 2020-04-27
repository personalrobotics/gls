#ifndef GLS_SELECTOR_BACKWARDSELECTOR_HPP_
#define GLS_SELECTOR_BACKWARDSELECTOR_HPP_

#include "gls/selector/Selector.hpp"

namespace gls {
namespace selector {

/// Selector that evaluates the edge on the path closest to the target.
class BackwardSelector : public Selector {
 public:
  /// Constructor.
  BackwardSelector();

  /// Documentation inherited.
  gls::datastructures::Edge selectEdgeToEvaluate(
      gls::datastructures::Path path) override;
};

}  // namespace selector
}  // namespace gls

#endif  // GLS_SELECTOR_BACKWARDSELECTOR_HPP_
