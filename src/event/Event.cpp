/* Authors: Aditya Vamsikrishna Mandalika */

#include "gls/event/Event.hpp"

namespace gls {
namespace event {

Event::Event(gls::datastructures::GraphPtr graph, gls::datastructures::Vertex source, gls::datastructures::Vertex target)
: mGraph(graph)
, mSourceVertex(source)
, mTargetVertex(target) 
{
  // Do nothing.
}

} // event
} // gls
