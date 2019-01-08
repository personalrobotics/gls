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
  /// \param[in] 
  Selector(gls::datastructures::GraphPtr graph, gls::datastructures::Vertex source, gls::datastructures::Vertex target);

  /// Destructor.
  ~Selector() = default;

  /// Selects edges to evaluate from given path.
  virtual void selectEdgesToEvaluate(gls::datastructures::PathPtr path) = 0;

protected:
  /// Ranks the edges.
  virtual void rankEdgesByUtilityInEvaluation() = 0;

  /// Pointer to the graph.
  gls::datastructures::GraphPtr mGraph;

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
