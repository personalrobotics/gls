#ifndef GLS_SELECTOR_FORWARDSELECTOR_HPP_
#define GLS_SELECTOR_FORWARDSELECTOR_HPP_

#include "gls/selector/Selector.hpp"

namespace gls {

/// Selector that evaluates the edge on the path closest to the source.
class ForwardSelector : public GLS::Selector {
 public:
  /// Constructor.
  ForwardSelector();

  /// Documentation inherited.
  Edge selectEdgeToEvaluate(Path path) override;
};

}  // namespace gls

#endif  // GLS_SELECTOR_FORWARDSELECTOR_HPP_
