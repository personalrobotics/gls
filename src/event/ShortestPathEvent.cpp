#include "gls/event/ShortestPathEvent.hpp"

namespace gls {
namespace event {

using gls::datastructures::Graph;
using gls::datastructures::SearchQueue;
using gls::datastructures::Vertex;

//==============================================================================
ShortestPathEvent::ShortestPathEvent() {
  // Do nothing.
}

//==============================================================================
bool ShortestPathEvent::isTriggered(const Vertex& vertex) {
  if(std::find(mTargetVertices.begin(), mTargetVertices.end(), vertex) != mTargetVertices.end()) {
    return true;
  }
  return false;
}

//==============================================================================
void ShortestPathEvent::updateVertexProperties(Vertex& /*vertex*/) {
  // Do nothing.
}

} // namespace event
} // namespace gls
