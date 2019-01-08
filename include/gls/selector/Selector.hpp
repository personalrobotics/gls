/* Authors: Aditya Vamsikrishna Mandalika */

#ifndef GLS_SELECTOR_SELECTOR_HPP_
#define GLS_SELECTOR_SELECTOR_HPP_

#include <string>  // std::string
#include <utility> // std::pait
#include <vector>  // std::vector

#include "gls/datastructures/Graph.hpp"
#include "gls/datastructures/Types.hpp"

namespace gls {
namespace selector {

/// Selector is a base class for selecting edges to evaluate.
/// The rule for selecting the edges to evaluate is specified
/// by the concrete classes.
class Selector
{
public:
  /// Constructor.
  /// \param[in] graph Graph the selector is operating with.
  /// \param[in] source Source vertex in the graph the selector is attached to.
  /// \param[in] target Target vertex in the graph the selector is attached to.
  Selector(
      gls::datastructures::Graph& graph,
      gls::datastructures::Vertex source,
      gls::datastructures::Vertex target);

  /// Destructor.
  ~Selector() = default;

  /// Selects edges to evaluate from given path.
  virtual gls::datastructures::Path selectEdgesToEvaluate(
      gls::datastructures::Path path)
      = 0;

protected:
  /// Ranks the edges.
  virtual void rankEdgesByUtilityInEvaluation() = 0;

  /// Reference to the graph.
  gls::datastructures::Graph& mGraph;

  /// Source vertex of the graph.
  gls::datastructures::Vertex mSourceVertex;

  /// Target vertex of the graph.
  gls::datastructures::Vertex mTargetVertex;

}; // Selector

typedef std::shared_ptr<Selector> SelectorPtr;
typedef std::shared_ptr<const Selector> ConstSelectorPtr;

} // selector
} // gls

#endif // GLS_SELECTOR_SELECTOR_HPP_
