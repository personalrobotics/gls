/* Authors: Aditya Vamsikrishna Mandalika */

#include "gls/selector/Selector.hpp"

namespace gls {

//==============================================================================
GLS::Selector::Selector() {
  // Do nothing.
}

//==============================================================================
void GLS::Selector::setup(Graph* graph) { mGraph = graph; }

}  // namespace gls
