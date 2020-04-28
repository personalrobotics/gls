#include "gls/event/ShortestPathEvent.hpp"

namespace gls {

//==============================================================================
ShortestPathEvent::ShortestPathEvent() {
  // Do nothing.
}

//==============================================================================
bool ShortestPathEvent::isTriggered(const Vertex& vertex) {
  if (vertex == mTargetVertex) {
    return true;
  }
  return false;
}

//==============================================================================
void ShortestPathEvent::updateVertexProperties(Vertex& /*vertex*/) {
  // Do nothing.
}

}  // namespace gls
