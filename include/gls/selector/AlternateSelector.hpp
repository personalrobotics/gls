#ifndef GLS_SELECTOR_ALTERNATESELECTOR_HPP_
#define GLS_SELECTOR_ALTERNATESELECTOR_HPP_

#include "gls/selector/Selector.hpp"

namespace gls {

/// Selector alternates between forward and backward selector.
/// By default the selector begins with forward and then flips.
class AlternateSelector : public GLS::Selector {
 public:
  /// Constructor.
  AlternateSelector();

  /// Documentation inherited.
  Edge selectEdgeToEvaluate(Path path) override;

 private:
  /// Iteration index to switch between forward and backward.
  bool mUseForwardSelector{false};

  /// Forward Selector.
  GLS::SelectorPtr mForwardSelector;

  /// Backward Selector.
  GLS::SelectorPtr mBackwardSelector;
};

}  // namespace gls

#endif  // GLS_SELECTOR_ALTERNATESELECTOR_HPP_
