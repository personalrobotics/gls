/* Authors: Aditya Vamsikrishna Mandalika */

#ifndef GLS_DATASTRUCTURES_GRAPH_HPP_
#define GLS_DATASTRUCTURES_GRAPH_HPP_

// STL headers
#include <vector>
#include <set>

// Boost headers
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>
#include <boost/property_map/dynamic_property_map.hpp>

// GLS headers
#include "GLS/Datastructures/Types.hpp"

namespace gls {
namespace datastructures {

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

  // Set the vertex parent.
  void setParent(Vertex parent);

  // Get the vertex parent.
  Vertex getParent();

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

private:
  /// Cost-to-Come.
  double mCostToCome;

  /// Heuristic value.
  double mHeuristic;

  /// Parent.
  Vertex mParent;

  /// Children.
  std::set<Vertex> mChildren;
};

class EdgeProperties
{
public:
  // Sets the length of the edge.
  void setLength(double length);

  // Get the length of the edge.
  double getLength();

  // Sets the edge to have been evaluated.
  void setEvaluationStatus(EvaluationStatus evaluationStatus);

  // Checks if the edge has been evaluated.
  EvaluationStatus getEvaluationStatus();

private:
  /// The length of the edge using the space distance metric
  double mLength;

  /// Flag to check if edge is evaluated
  EvaluationStatus mEvaluationStatus;
};

/// Undirected Boost graph using the properties just defined.
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, VertexProperties, EdgeProperties> Graph;

/// Shared pointer to Graph.
typedef std::shared_ptr<Graph> GraphPtr;

/// Shared pointer to const Graph.
typedef std::shared_ptr<const Graph> ConstGraphPtr;

} // datastructures
} // gls

#endif // GLS_DATASTRUCTURES_GRAPH_HPP_
