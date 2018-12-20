/* Authors: Aditya Vamsikrishna Mandalika */

#ifndef GLS_SELECTOR_SELECTOR_HPP_
#define GLS_SELECTOR_SELECTOR_HPP_

#include <string>   // std::string
#include <utility>  // std::pait
#include <vector>   // std::vector

namespace gls {
namespace selector {

/// Selector is a base class for selecting edges to evaluate.
/// The rule for selecting the edges to evaluate is specified
/// by the concrete classes.
class Selector
{
public:
  /// Constructor.
  Selector();

  /// Destructor.
  ~Selector() = default;

  /// Selects edges to evaluate from given path.
  virtual void selectEdgesToEvaluate(gls::datastructures::PathPtr path) = 0;

protected:
  /// Ranks the edges.
  virtual void rankEdgesByUtilityInEvaluation() = 0;

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
