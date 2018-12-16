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
  virtual ~Selector() = default;

  // Selects edges to evaluate from given set of edges.
  // TODO (avk): Is vector the right datastructure.
  // By default only support selecting one edge.
  void selectEdgesToEvaluate(std::vector<std::size_t>& edges);

protected:
  // Ranks the edges from 
  virtual void rankEdgesByUtilityInEvaluation() = 0;

  // Data Member to hold the edges. Why?
}; // Selector

} // selector
} // gls

#endif // GLS_SELECTOR_SELECTOR_HPP_
