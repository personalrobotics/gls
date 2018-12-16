/* Authors: Aditya Vamsikrishna Mandalika */

#ifndef GLS_DATASTRUCTURES_GRAPH_HPP_
#define GLS_DATASTRUCTURES_GRAPH_HPP_

// STL headers
#include <vector>

// Boost headers
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>
#include <boost/property_map/dynamic_property_map.hpp>

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
  double getCostToCome() const;

  // Set heuristic.
  void setHeuristic(double heuristic);

  // Get heuristic.
  double getHeuristic() const;

  // Set the vertex parent.
  void setParent(Vertex parent);

  // Get the vertex parent.
  Vertex getParent() const;

  // Set the children of the vertex in the search tree.
  void setChildren(std::vector<Vertex> children);

  // Add a single child to the vertex.
  void addChild(Vertex child);

  // Add multiple children to the vertex.
  void addChildren(std::vector<Vertex> children);

  // Remove a single child.
  void removeChild(Vertex child);

  // Remove multiple children.
  void removeChildren(std::vector<Vertex> children);

  // Clears the children.
  void removeAllChildren();

  // Get children.
  std::vector<Vertex>& getChildren() const;

  // Checks if vertex has given child.
  bool hasChild(Vertex child) const;

private:
  /// Cost-to-Come.
  double mCostToCome;

  /// Heuristic value.
  double mHeuristic;

  /// Parent.
  Vertex mParent;

  /// Children.
  std::vector<Vertex> mChildren;
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
  double length;

  /// Flag to check if edge is evaluated
  bool isEvaluated;
};

/// Undirected Boost graph
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, VertexProperties, EdgeProperties> Graph;

} // datastructures
} // gls

#endif // GLS_DATASTRUCTURES_GRAPH_HPP_
