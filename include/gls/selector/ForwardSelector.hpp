#ifndef GLS_SELECTOR_FORWARDSELECTOR_HPP_
#define GLS_SELECTOR_FORWARDSELECTOR_HPP_

#include "gls/selector/Selector.hpp"

namespace gls {
namespace selector {

/// Selector that evaluates the edge on the path closest to the source.
class ForwardSelector : public Selector
{
public:
  /// Constructor.
  ForwardSelector();

  /// Documentation inherited.
  gls::datastructures::Path selectEdgesToEvaluate(
      gls::datastructures::Path path) override;
};

} // namespace selector
} // namespace gls

#endif // GLS_SELECTOR_FORWARDSELECTOR_HPP_
