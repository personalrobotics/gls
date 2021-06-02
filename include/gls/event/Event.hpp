/* Authors: Aditya Vamsikrishna Mandalika */

#ifndef GLS_EVENT_EVENT_HPP_
#define GLS_EVENT_EVENT_HPP_

#include <string>  // std::string
#include <utility> // std::pair
#include <vector>  // std::vector

#include "gls/datastructures/Graph.hpp"
#include "gls/datastructures/SearchQueue.hpp"
//#include "gls/datastructures/Types.hpp"

namespace gls {
namespace event {

enum vertexUpdateOption { SingleUpdate, CascadeUpdate };

/// Event is a base class to define the trigger to pause search.
/// The rule for switching between serach and edge evaluation is
/// specified by the concrete classes.
class Event {
public:
  /// Constructor.
  Event();

  /// Destructor.
  virtual ~Event() = default;

  /// Setup the event with required internal data members.
  /// \param[in] graph Graph the event is operating with.
  /// \param[in] source Source vertex in the graph the event is attached to.
  /// \param[in] target Target vertex in the graph the event is attached to.
  void setup(
      gls::datastructures::Graph* graph,
      gls::datastructures::Vertex& source,
      gls::datastructures::Vertex& target);

  /// Return true if the event is triggered.
  /// \param[in] vertex Vertex that might cause the trigger.
  virtual bool isTriggered(const gls::datastructures::Vertex& vertex) = 0;

  /// Update vertex properties
  /// Concrete classes specify the appropriate update rules.
  /// \param[in] vertex Vertex whose properties need to be updated.
  /// downstream.
  virtual void updateVertexProperties(gls::datastructures::Vertex& vertex) = 0;

  /// Update vertex properties
  /// Updates the tree by recursively calling update on each vertex.
  /// \param[in] updateQueue Queue of vertices whose subtrees need update.
  void updateVertexProperties(gls::datastructures::SearchQueue& updateQueue);

protected:
  /// Pointer to the graph.
  gls::datastructures::Graph* mGraph;

  /// Source vertex of the graph.
  gls::datastructures::Vertex mSourceVertex;

  /// Target vertex of the graph.
  gls::datastructures::Vertex mTargetVertex;

}; // Event

typedef std::shared_ptr<Event> EventPtr;
typedef std::shared_ptr<const Event> ConstEventPtr;

} // namespace event
} // namespace gls

#endif // GLS_EVENT_EVENT_HPP_
