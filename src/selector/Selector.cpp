/* Authors: Aditya Vamsikrishna Mandalika */

#include "gls/selector/Selector.hpp"

namespace gls {
namespace selector {

using gls::datastructures::GraphPtr;
using gls::datastructures::Vertex;

//==============================================================================
Selector::Selector(GraphPtr graph, Vertex source, Vertex target)
  : mGraph(graph), mSourceVertex(source), mTargetVertex(target)
{
  // Do nothing.
}

} // selector
} // gls
