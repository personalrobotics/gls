#ifndef GLS_EVENT_CONSTANTDEPTHEVENT_HPP_
#define GLS_EVENT_CONSTANTDEPTHEVENT_HPP_

#include <unordered_map>

#include "gls/event/Event.hpp"

namespace gls {

/// Event that triggers when the search tree reaches a particular depth.
/// Additionally, the event also triggers when the vertex is the target.
class ConstantDepthEvent : public GLS::Event {
 public:
  /// Constructor.
  explicit ConstantDepthEvent(std::size_t depth);

  /// Documentation inherited.
  bool isTriggered(const Vertex& vertex) override;

  /// Documentation inherited.
  void updateVertexProperties(Vertex& vertex) override;

 private:
  /// Get the depth of the vertex.
  std::size_t getDepth(const Vertex& vertex);

  /// Add vertex to the map.
  void updateVertexInMap(Vertex& vertex);

  /// The threshold over depth.
  std::size_t mDepthThreshold;

  /// The map from vertex to depth in the search tree.
  std::unordered_map<Vertex, std::size_t> mVertexDepthMap;
};

}  // namespace gls

#endif  // GLS_EVENT_CONSTANTDEPTHEVENT_HPP_
