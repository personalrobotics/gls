/* Authors: Aditya Vamsikrishna Mandalika */

#include "gls/event/Event.hpp"

namespace gls {
namespace event {

using gls::datastructures::GraphPtr;
using gls::datastructures::Vertex;

//==============================================================================
Event::Event(GraphPtr graph, Vertex source, Vertex target)
  : mGraph(graph), mSourceVertex(source), mTargetVertex(target)
{
  // Do nothing.
}

} // event
} // gls
