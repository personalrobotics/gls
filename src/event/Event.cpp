/* Authors: Aditya Vamsikrishna Mandalika */

#include "gls/event/Event.hpp"

namespace gls {
namespace event {

using gls::datastructures::Graph;
using gls::datastructures::Vertex;

//==============================================================================
Event::Event()
{
  // Do nothing.
}

//==============================================================================
void Event::setup(Graph& graph, Vertex source, Vertex target)
{
  mGraph = graph;
  mSourceVertex = source; 
  mTargetVertex = target;
}

} // event
} // gls
