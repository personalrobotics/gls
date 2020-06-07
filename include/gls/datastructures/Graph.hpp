// Author: Aditya Vamsikrishna Mandalika
// Date: 6th June 2020

#ifndef GLS_DATASTRUCTURES_GRAPH_HPP_
#define GLS_DATASTRUCTURES_GRAPH_HPP_

namespace gls {

/// \brief Node corresponding to a vertex in the graph.
struct Node {
  /// Node ID.
  std::size_t id;

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

  /// Returns the total cost of the node.
  constexpr double totalCost() const { return costToCome + heuristic; };
};

}  // namespace gls

#endif  // GLS_DATASTRUCTURES_GRAPH_HPP_
