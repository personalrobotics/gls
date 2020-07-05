// Author: Aditya Vamsikrishna Mandalika
// Date: 6th June 2020

#ifndef GLS_DATASTRUCTURES_GRAPH_HPP_
#define GLS_DATASTRUCTURES_GRAPH_HPP_

#include <ompl/base/State.h>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>
#include <boost/property_map/dynamic_property_map.hpp>

namespace gls {

/// \brief Represents the node properties in the graph.
struct NodeProp {
  /// Node ID.
  std::size_t id;

  /// The underlying state of the node.
  ompl::base::State* state;

  /// Cost-to-Come.
  double costToCome;

  /// Heuristic.
  double heuristic;

  /// Flag indicates if the node is in collision.
  bool collision;

  /// Flag indicates if the node has been expanded before.
  bool expanded;

  /// Flag indicates if the node is in repair.
  bool repair;

  /// Children in the search tree.
  std::vector<std::size_t> children;

  /// Returns the total cost of the node.
  constexpr double totalCost() const { return costToCome + heuristic; };

  /// Utility function to remove a child from \c children.
  inline void removeChild(const std::size_t& node) {
    for (auto i = 0u; i < children.size(); ++i) {
      if (children[i] == node) {
        children[i] = children.back();
        children.pop_back();
      }
    }
  }
};

/// \brief Represents the edge properties in the graph.
struct EdgeProp {
  /// Edge length.
  double length;

  /// Collision status of the edge.
  bool collision;

  /// Evaluation status of the edge.
  bool evaluated;
};

/// \brief Explicit boost graph representation.
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
                              NodeProp, EdgeProp>
    Graph;

/// Aliases for code simplification.
/// Unique pointer to the graph. It is expected that only the planner changes
/// graph properties and has ownership.
typedef std::unique_ptr<Graph> GraphPtr;

/// Boost vertex iterator
typedef boost::graph_traits<Graph>::vertex_iterator VertexIterator;

/// Boost edge iterator
typedef boost::graph_traits<Graph>::edge_iterator EdgeIterator;

/// Boost graph neighbor iterator
typedef boost::graph_traits<Graph>::adjacency_iterator NeighborIterator;

/// Map each vertex to the underlying OMPL state in the node.
typedef boost::property_map<Graph, ompl::base::State * NodeProp::*>::type
    VertexStateMap;

/// Map each edge to its length.
typedef boost::property_map<Graph, double EdgeProp::*>::type EdgeLengthMap;

}  // namespace gls

#endif  // GLS_DATASTRUCTURES_GRAPH_HPP_
