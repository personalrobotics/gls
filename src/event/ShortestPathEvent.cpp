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
bool ShortestPathEvent::isTriggered(const Vertex vertex)
{
  if (vertex == mTargetVertex)
    return true;

  return false;
}

//==============================================================================
void ShortestPathEvent::updateVertexProperties(Vertex /*vertex*/)
{
  // Do nothing.
}

//==============================================================================
void ShortestPathEvent::updateVertexProperties(SearchQueue& queue)
{
	// Clear the queue since there is nothing to update.
	queue.clear();
}

} // namespace event
} // namespace gls
