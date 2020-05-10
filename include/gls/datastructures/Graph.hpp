/* Authors: Aditya Vamsikrishna Mandalika */

#ifndef GLS_DATASTRUCTURES_GRAPH_HPP_
#define GLS_DATASTRUCTURES_GRAPH_HPP_

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>
#include <boost/property_map/dynamic_property_map.hpp>
#include <set>
#include <vector>

#include "gls/GLS.hpp"
#include "gls/datastructures/SearchQueue.hpp"
#include "gls/datastructures/Types.hpp"

// TODO (avk): state and length are made public to accomodate the
// roadmapmanager which seems stupid. Change it if possible.

namespace gls {

class GLS::VertexProperties {
 public:
  // Set state wrapper around underlying OMPL state.
  void setState(StatePtr state);

  // Get state wrapper around underlying OMPL state.
  StatePtr getState();

  // Set cost-to-come.
  void setCostToCome(double cost);

  // Get cost-to-come.
  double getCostToCome();

  // Set heuristic.
  void setHeuristic(double heuristic);

  // Get heuristic.
  double getHeuristic();

  // Get estimated total cost.
  double getEstimatedTotalCost();

  // Set the vertex parent.
  void setParent(Vertex parent);

  // Get the vertex parent.
  Vertex getParent();

  // Get the set of all children.
  // TODO (avk): Remove this with getDescendents.
  // Since this is only used in a loop.
  std::set<Vertex>& getChildren();

  // Set the children of the vertex in the search tree.
  void setChildren(std::set<Vertex> children);

  // Add a single child to the vertex.
  void addChild(Vertex child);

  // Add multiple children to the vertex.
  void addChildren(std::set<Vertex> children);

  // Remove a single child.
  void removeChild(Vertex child);

  // Remove multiple children.
  void removeChildren(std::set<Vertex> children);

  // Clears the children.
  void removeAllChildren();

  // Checks if vertex has given child.
  bool hasChild(Vertex child);

  // Sets the visit status of the vertex.
  void setVisitStatus(VisitStatus status);

  // Get the visit status of the vertex.
  VisitStatus getVisitStatus();

  // Sets the collision status of the vertex.
  void setCollisionStatus(CollisionStatus status);

  // Get the collision status of the vertex.
  CollisionStatus getCollisionStatus();

  // Sets the iterator to the vertex in the search queue.
  void setSearchIterator(const SearchQueue::SearchQueueIterator);

  /// \brief The iterator to the vertex in the queue.
  SearchQueue::SearchQueueIterator getSearchIterator() const;

  /// \brief Clear the iterator-set flag.
  void clearSearchIterator();

  /// \brief Indicates if current vertex is in search queue.
  bool inSearchQueue() const;

  /// Underlying state.
  /// TODO (avk): Preferable if this was private.
  StatePtr mState;

 private:
  /// Cost-to-Come.
  double mCostToCome{std::numeric_limits<double>::infinity()};

  /// Heuristic value.
  double mHeuristic{std::numeric_limits<double>::infinity()};

  /// Parent.
  Vertex mParent;

  /// Children.
  std::set<Vertex> mChildren;

  /// Visitation status.
  VisitStatus mVisitStatus{VisitStatus::NotVisited};

  /// Collision status.
  CollisionStatus mCollisionStatus{CollisionStatus::Free};

  /// Iterator in the search queue.
  GLS::SearchQueue::SearchQueueIterator mSearchIterator;

  /// Flag indicating if the vertex is in the search queue.
  /// This flag is updated whenever \c mSearchIterator is updated.
  bool mInSearchQueue{false};
};

class GLS::EdgeProperties {
 public:
  // Sets the length of the edge.
  void setLength(double length);

  // Get the length of the edge.
  double getLength();

  // Sets the evaluation status.
  void setEvaluationStatus(EvaluationStatus evaluationStatus);

  // Get the evaluation status.
  EvaluationStatus getEvaluationStatus();

  // Sets the collision status.
  void setCollisionStatus(CollisionStatus status);

  // Get the collision status.
  CollisionStatus getCollisionStatus();

  /// The length of the edge using the space distance metric.
  /// TODO (avk): Why is this public?
  double mLength;

 private:
  /// Evaluation status.
  EvaluationStatus mEvaluationStatus{EvaluationStatus::NotEvaluated};

  /// Collision status..
  CollisionStatus mCollisionStatus{CollisionStatus::Free};
};

using VPStateMap =
    boost::property_map<GLS::Graph,
                        GLS::StatePtr GLS::VertexProperties::*>::type;
using EPLengthMap =
    boost::property_map<GLS::Graph, double GLS::EdgeProperties::*>::type;

/// Boost vertex iterator
typedef boost::graph_traits<GLS::Graph>::vertex_iterator VertexIter;

/// Boost edge iterator
typedef boost::graph_traits<GLS::Graph>::edge_iterator EdgeIter;

/// Boost graph neighbor iterator
typedef boost::graph_traits<GLS::Graph>::adjacency_iterator NeighborIter;

}  // namespace gls

#endif  // GLS_DATASTRUCTURES_GRAPH_HPP_
