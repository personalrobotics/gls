/* Authors: Aditya Vamsikrishna Mandalika */

#ifndef GLS_EVENT_EVENT_HPP_
#define GLS_EVENT_EVENT_HPP_

#include <string>  // std::string
#include <utility> // std::pair
#include <vector>  // std::vector

#include "gls/datastructures/Graph.hpp"
#include "gls/datastructures/Types.hpp"

namespace gls {
namespace event {

enum vertexUpdateOption
{
  SingleUpdate,
  CascadeUpdate
};

/// Event is a base class to define the trigger to pause search.
/// The rule for switching between serach and edge evaluation is
/// specified by the concrete classes.
class Event
{
public:
  /// Constructor.
  /// \param[in] graph Graph the event is operating with.
  /// \param[in] source Source vertex in the graph the event is attached to.
  /// \param[in] target Target vertex in the graph the event is attached to.
  Event(
      gls::datastructures::GraphPtr graph,
      gls::datastructures::Vertex source,
      gls::datastructures::Vertex target);

  /// Destructor.
  virtual ~Event() = default;

  /// Return true if the event is triggered.
  /// \param[in] vertex Vertex that might cause the trigger.
  virtual bool isTriggered(const gls::datastructures::Vertex vertex) const = 0;

  /// Update vertex properties
  /// Concrete classes specify the appropriate update rules.
  /// \param[in] vertex Vertex whose properties need to be updated.
  /// \param[in] cascade Set to true if the update needs to be cascaded
  /// downstream.
  virtual void updateVertexProperties(
      gls::datastructures::Vertex vertex,
      vertexUpdateOption cascade = vertexUpdateOption::CascadeUpdate)
      = 0;

protected:
  /// Pointer to the graph.
  gls::datastructures::GraphPtr mGraph;

  /// Source vertex of the graph.
  gls::datastructures::Vertex mSourceVertex;

  /// Target vertex of the graph.
  gls::datastructures::Vertex mTargetVertex;

}; // Event

typedef std::shared_ptr<Event> EventPtr;
typedef std::shared_ptr<const Event> ConstEventPtr;

} // event
} // gls

#endif // GLS_EVENT_EVENT_HPP_
