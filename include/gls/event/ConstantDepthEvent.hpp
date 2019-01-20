#ifndef GLS_EVENT_CONSTANTDEPTHEVENT_HPP_
#define GLS_EVENT_CONSTANTDEPTHEVENT_HPP_

#include <unordered_map>

#include "gls/event/Event.hpp"

namespace gls {
namespace event {

/// Event that triggers when the search tree reaches a particular depth.
class ConstantDepthEvent : public Event
{
public:
  /// Constructor.
  explicit ConstantDepthEvent(double depth);

  /// Documentation inherited.
  bool isTriggered(const gls::datastructures::Vertex vertex) const override;

  /// Documentation inherited.
  void updateVertexProperties(
      gls::datastructures::Vertex vertex,
      vertexUpdateOption cascade = vertexUpdateOption::SingleUpdate) override;

  /// Documentation inherited.
  void updateVertexProperties(
      gls::datastructures::SearchQueue vertexQueue) override;

private:
  /// Get the depth of the vertex.
  double getDepth(gls::datastructures::Vertex vertex);

  /// Add vertex to the map.
  void addVertexToMap(gls::datastructures::Vertex vertex);

  /// The threshold over depth.
  double mDepthThreshold;

  /// The map from vertex to depth in the search tree.
  std::unordered_map<gls::datastructures::Vertex, double> mVertexDepthMap;
};

} // namespace event
} // namespace gls

#endif // GLS_EVENT_CONSTANTDEPTHEVENT_HPP_
