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
  /// \param[in] graph Graph the event is operating with.
  /// \param[in] source Source vertex in the graph the event is attached to.
  /// \param[in] target Target vertex in the graph the event is attached to.
  ShortestPathEvent(
      gls::datastructures::GraphPtr graph,
      gls::datastructures::Vertex source,
      gls::datastructures::Vertex target);

  /// Documentation inherited.
  bool isTriggered(const gls::datastructures::Vertex vertex) const override;

  void updateVertexProperties(
      gls::datastructures::Vertex vertex,
      vertexUpdateOption cascade = vertexUpdateOption::CascadeUpdate) override;
};

} // namespace event
} // namespace gls

#endif // GLS_EVENT_SHORTESTPATHEVENT_HPP_
