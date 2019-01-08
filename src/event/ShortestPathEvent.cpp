#include "gls/event/ShortestPathEvent.hpp"

namespace gls {
namespace event {

using gls::datastructures::Graph;
using gls::datastructures::Vertex;

//==============================================================================
ShortestPathEvent::ShortestPathEvent(
    Graph& graph, Vertex source, Vertex target)
  : Event(graph, source, target)
{
  // Do nothing.
}

//==============================================================================
bool ShortestPathEvent::isTriggered(const Vertex vertex) const
{
  if (vertex == mTargetVertex)
    return true;

  return false;
}

//==============================================================================
void ShortestPathEvent::updateVertexProperties(
    Vertex vertex, vertexUpdateOption cascade)
{
  // Do nothing.
}

} // namespace event
} // namespace gls
