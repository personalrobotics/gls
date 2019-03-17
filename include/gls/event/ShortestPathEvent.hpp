#ifndef GLS_EVENT_SHORTESTPATHEVENT_HPP_
#define GLS_EVENT_SHORTESTPATHEVENT_HPP_

#include "gls/event/Event.hpp"

namespace gls {
namespace event {

/// Event that triggers when a shortest path to goal is found.
class ShortestPathEvent : public Event
{
public:
  /// Constructor.
  ShortestPathEvent();

  /// Documentation inherited.
  bool isTriggered(const gls::datastructures::Vertex vertex) override;

  /// Documentation inherited.
  void updateVertexProperties(gls::datastructures::Vertex vertex) override;

  /// Documentation inherited.
  void updateVertexProperties(
      gls::datastructures::SearchQueue& vertexQueue) override;
};

} // namespace event
} // namespace gls

#endif // GLS_EVENT_SHORTESTPATHEVENT_HPP_
