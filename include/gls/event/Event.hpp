/* Authors: Aditya Vamsikrishna Mandalika */

#ifndef GLS_EVENT_EVENT_HPP_
#define GLS_EVENT_EVENT_HPP_

#include <string>
#include <utility>
#include <vector>

#include "gls/GLS.hpp"
#include "gls/datastructures/Types.hpp"

namespace gls {

enum vertexUpdateOption { SingleUpdate, CascadeUpdate };

/// Event is a base class to define the trigger to pause search.
/// The rule for switching between serach and edge evaluation is
/// specified by the concrete classes.
class GLS::Event {
 public:
  /// Constructor.
  Event();

  /// Destructor.
  virtual ~Event() = default;

  /// Setup the event with required internal data members.
  /// \param[in] graph Graph the event is operating with.
  /// \param[in] source Source vertex in the graph the event is attached to.
  /// \param[in] target Target vertex in the graph the event is attached to.
  void setup(Graph* graph, Vertex& source, Vertex& target);

  /// Return true if the event is triggered.
  /// \param[in] vertex Vertex that might cause the trigger.
  virtual bool isTriggered(const Vertex& vertex) = 0;

  /// Update vertex properties
  /// Concrete classes specify the appropriate update rules.
  /// \param[in] vertex Vertex whose properties need to be updated.
  /// downstream.
  virtual void updateVertexProperties(Vertex& vertex) = 0;

  /// Update vertex properties
  /// Updates the tree by recursively calling update on each vertex.
  /// \param[in] updateQueue Queue of vertices whose subtrees need update.
  void updateVertexProperties(SearchQueuePtr updateQueue);

 protected:
  /// Pointer to the graph.
  Graph* mGraph;

  /// Source vertex of the graph.
  Vertex mSourceVertex;

  /// Target vertex of the graph.
  Vertex mTargetVertex;

};  // Event

}  // namespace gls

#endif  // GLS_EVENT_EVENT_HPP_
