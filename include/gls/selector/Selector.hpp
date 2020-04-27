/* Authors: Aditya Vamsikrishna Mandalika */

#ifndef GLS_SELECTOR_SELECTOR_HPP_
#define GLS_SELECTOR_SELECTOR_HPP_

#include <string>   // std::string
#include <utility>  // std::pait
#include <vector>   // std::vector

#include "gls/datastructures/Graph.hpp"
#include "gls/datastructures/Types.hpp"

namespace gls {
namespace selector {

/// Selector is a base class for selecting edges to evaluate.
/// The rule for selecting the edges to evaluate is specified
/// by the concrete classes.
class Selector {
 public:
  /// Constructor.
  Selector();

  /// Destructor.
  ~Selector() = default;

  /// Setup the selector with required internal data members.
  /// \param[in] graph Graph the selector is operating with.
  void setup(gls::datastructures::Graph* graph);

  /// Selects edges to evaluate from given path.
  /// \param[in] path The list of vertices along the path.
  /// The vertices are from leaf to source.
  virtual gls::datastructures::Edge selectEdgeToEvaluate(
      gls::datastructures::Path path) = 0;

 protected:
  /// Pointer to the graph.
  gls::datastructures::Graph* mGraph;

};  // Selector

typedef std::shared_ptr<Selector> SelectorPtr;
typedef std::shared_ptr<const Selector> ConstSelectorPtr;

}  // namespace selector
}  // namespace gls

#endif  // GLS_SELECTOR_SELECTOR_HPP_
