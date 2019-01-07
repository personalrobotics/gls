/* Authors: Aditya Vamsikrishna Mandalika */

#ifndef GLS_DATASTRUCTURES_GRAPH_HPP_
#define GLS_DATASTRUCTURES_GRAPH_HPP_

// STL headers
#include <set>
#include <vector>

// Boost headers
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>
#include <boost/property_map/dynamic_property_map.hpp>

// GLS headers
#include "gls/datastructures/State.hpp"
#include "gls/datastructures/Types.hpp"

namespace gls {
namespace datastructures {

enum CollisionStatus
{
  Collision,
  Free
};

enum VisitStatus
{
  NotVisited,
  Visited
};

enum EvaluationStatus
{
  NotEvaluated,
  Evaluated
};

class VertexProperties
{
public:
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

private:
  /// Cost-to-Come.
  double mCostToCome;

  /// Heuristic value.
  double mHeuristic;

  /// Parent.
  Vertex mParent;

  /// Children.
  std::set<Vertex> mChildren;

  /// Visitation status.
  VisitStatus mVisitStatus;

  /// Collision status.
  CollisionStatus mCollisionStatus;
};

class EdgeProperties
{
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

private:
  /// The length of the edge using the space distance metric.
  double mLength;

  /// Evaluation status.
  EvaluationStatus mEvaluationStatus;

  /// Collision status..
  CollisionStatus mCollisionStatus;
};

/// Undirected Boost graph using the properties just defined.
typedef boost::adjacency_list<boost::vecS,
                              boost::vecS,
                              boost::undirectedS,
                              VertexProperties,
                              EdgeProperties>
    Graph;

/// Shared pointer to Graph.
typedef std::shared_ptr<Graph> GraphPtr;

/// Shared pointer to const Graph.
typedef std::shared_ptr<const Graph> ConstGraphPtr;

/// Boost graph neighbor iterator
typedef boost::graph_traits<Graph>::adjacency_iterator NeighborIter;

/// Map each vertex to the underlying state [read from the graphml file]
typedef boost::property_map<Graph, gls::datastructures::StatePtr VertexProperties::*>::type VPStateMap;

/// Map each edge to its length
typedef boost::property_map<Graph, double EdgeProperties::*>::type EPLengthMap;

} // datastructures
} // gls

#endif // GLS_DATASTRUCTURES_GRAPH_HPP_
