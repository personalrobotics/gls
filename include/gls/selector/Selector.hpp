/* Authors: Aditya Vamsikrishna Mandalika */

#ifndef GLS_SELECTOR_SELECTOR_HPP_
#define GLS_SELECTOR_SELECTOR_HPP_

#include <string>
#include <utility>
#include <vector>

#include "gls/GLS.hpp"
#include "gls/datastructures/Types.hpp"

namespace gls {

/// Selector is a base class for selecting edges to evaluate.
/// The rule for selecting the edges to evaluate is specified
/// by the concrete classes.
class GLS::Selector {
 public:
  /// Constructor.
  Selector();

  /// Destructor.
  ~Selector() = default;

  /// Setup the selector with required internal data members.
  /// \param[in] graph Graph the selector is operating with.
  void setup(Graph* graph);

  /// Selects edges to evaluate from given path.
  /// \param[in] path The list of vertices along the path.
  /// The vertices are from leaf to source.
  virtual Edge selectEdgeToEvaluate(Path path) = 0;

 protected:
  /// Pointer to the graph.
  Graph* mGraph;

};  // Selector

}  // namespace gls

#endif  // GLS_SELECTOR_SELECTOR_HPP_
