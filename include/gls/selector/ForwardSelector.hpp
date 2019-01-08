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
  /// \param[in] graph Graph the selector is operating with.
  /// \param[in] source Source vertex in the graph the selector is attached to.
  /// \param[in] target Target vertex in the graph the selector is attached to.
  ForwardSelector(
      gls::datastructures::Graph& graph,
      gls::datastructures::Vertex source,
      gls::datastructures::Vertex target);

  /// Documentation inherited.
  gls::datastructures::Path selectEdgesToEvaluate(
      gls::datastructures::Path path) override;

  /// Documentation inherited.
  void rankEdgesByUtilityInEvaluation() override;
};

} // namespace selector
} // namespace gls

#endif // GLS_SELECTOR_FORWARDSELECTOR_HPP_
