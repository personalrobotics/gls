#ifndef GLS_EVENT_SUBPATHEXISTENCEEVENT_HPP_
#define GLS_EVENT_SUBPATHEXISTENCEEVENT_HPP_

#include <unordered_map>

#include "gls/event/Event.hpp"

namespace gls {
namespace event {

/// Unordered map defined as: <source name> <target name> <prior>
typedef std::unordered_map<
    std::pair<std::size_t, std::size_t>,
    double,
    boost::hash<std::pair<std::size_t, std::size_t>>>
    edgeToPriorMap;

/// Event that triggers when the search tree reaches below given
/// a threshold in probability of existence.
/// Additionally, the event also triggers when the vertex is the target.
class SubPathExistenceEvent : public Event {
public:
  /// Constructor.
  SubPathExistenceEvent(edgeToPriorMap& priorMap, double existenceThreshold);

  /// Documentation inherited.
  bool isTriggered(const gls::datastructures::Vertex vertex) override;

  /// Documentation inherited.
  void updateVertexProperties(gls::datastructures::Vertex vertex) override;

private:
  /// Get the probability of the path to the vertex.
  double getExistenceProbability(gls::datastructures::Vertex vertex);

  /// Add vertex to the map.
  void addVertexToMap(gls::datastructures::Vertex vertex);

  /// Evaluate the prior of given edge.
  double getPrior(gls::datastructures::Edge edge);

  /// Map that stores the priors.
  edgeToPriorMap mPriorMap;

  /// The threshold over depth.
  std::size_t mExistenceThreshold;

  /// The map from vertex to depth in the search tree.
  std::unordered_map<gls::datastructures::Vertex, double> mSubPathExistenceMap;
};

} // namespace event
} // namespace gls

#endif // GLS_EVENT_SUBPATHEXISTENCEEVENT_HPP_
