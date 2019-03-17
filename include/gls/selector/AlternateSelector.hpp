#ifndef GLS_SELECTOR_ALTERNATESELECTOR_HPP_
#define GLS_SELECTOR_ALTERNATESELECTOR_HPP_

#include "gls/selector/Selector.hpp"

namespace gls {
namespace selector {

/// Selector alternates between forward and backward selector.
/// By default the selector begins with forward and then flips.
class AlternateSelector : public Selector
{
public:
  /// Constructor.
  AlternateSelector();

  /// Documentation inherited.
  gls::datastructures::Path selectEdgesToEvaluate(
      gls::datastructures::Path path) override;

private:
  /// Iteration index to switch between forward and backward.
  bool mUseForwardSelector{false};

  /// Forward Selector.
  gls::selector::SelectorPtr mForwardSelector;

  /// Backward Selector.
  gls::selector::SelectorPtr mBackwardSelector;
};

} // namespace selector
} // namespace gls

#endif // GLS_SELECTOR_ALTERNATESELECTOR_HPP_
