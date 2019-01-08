/* Authors: Aditya Vamsikrishna Mandalika */

#include "gls/selector/Selector.hpp"

namespace gls {
namespace selector {

using gls::datastructures::Graph;
using gls::datastructures::Vertex;

//==============================================================================
Selector::Selector(Graph& graph, Vertex source, Vertex target)
  : mGraph(graph), mSourceVertex(source), mTargetVertex(target)
{
  // Do nothing.
}

} // selector
} // gls
