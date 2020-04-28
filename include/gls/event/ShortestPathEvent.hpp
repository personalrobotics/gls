#ifndef GLS_EVENT_SHORTESTPATHEVENT_HPP_
#define GLS_EVENT_SHORTESTPATHEVENT_HPP_

#include "gls/event/Event.hpp"

namespace gls {

/// Event that triggers when a shortest path to goal is found.
class ShortestPathEvent : public GLS::Event {
 public:
  /// Constructor.
  ShortestPathEvent();

  /// Documentation inherited.
  bool isTriggered(const Vertex& vertex) override;

  /// Documentation inherited.
  void updateVertexProperties(Vertex& vertex) override;
};

}  // namespace gls

#endif  // GLS_EVENT_SHORTESTPATHEVENT_HPP_
