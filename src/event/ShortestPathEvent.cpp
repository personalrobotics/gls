#include "gls/event/ShortestPathEvent.hpp"

namespace gls {
namespace event {

using gls::datastructures::GraphPtr;
using gls::datastructures::Vertex;

//==============================================================================
ShortestPathEvent::ShortestPathEvent(
    GraphPtr graph, Vertex source, Vertex target)
  : Event(graph, source, target)
{
  // Do nothing.
}

//==============================================================================
bool ShortestPathEvent::isTriggered(const Vertex vertex) const
{
  return true;
}

//==============================================================================
void ShortestPathEvent::updateVertexProperties(
    Vertex vertex, vertexUpdateOption cascade)
{
  // Do nothing.
}

} // namespace event
} // namespace gls
