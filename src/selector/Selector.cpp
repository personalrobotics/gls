/* Authors: Aditya Vamsikrishna Mandalika */

#include "gls/selector/Selector.hpp"

namespace gls {
namespace selector {

using gls::datastructures::Graph;

//==============================================================================
Selector::Selector() {
  // Do nothing.
}

//==============================================================================
void Selector::setup(Graph* graph) {
  mGraph = graph;
}

} // namespace selector
} // namespace gls
