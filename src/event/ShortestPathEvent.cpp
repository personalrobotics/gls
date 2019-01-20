#include "gls/event/ShortestPathEvent.hpp"

namespace gls {
namespace event {

using gls::datastructures::Graph;
using gls::datastructures::Vertex;
using gls::datastructures::SearchQueue;

//==============================================================================
ShortestPathEvent::ShortestPathEvent()
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
    Vertex /*vertex*/, vertexUpdateOption /*cascade*/)
{
  // Do nothing.
}

//==============================================================================
void ShortestPathEvent::updateVertexProperties(SearchQueue /*queue*/)
{
  // Do nothing.
}

} // namespace event
} // namespace gls
