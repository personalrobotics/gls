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

  /// Children in the search tree.
  std::vector<std::size_t> children;

  /// Returns the total cost of the node.
  constexpr double totalCost() const { return costToCome + heuristic; };

  /// Utility function to remove a child from \c children.
  inline void removeChild(const std::size_t& id) {
    for (auto i = 0; i < children.size(); ++i) {
      if (children[i] == id) {
        children[i] = children.back();
        children.pop_back();
      }
    }
  }
};

/// \brief Represents the edge properties in the graph.
struct Edge {
  /// Edge length.
  double length;

  /// Collision status of the edge.
  bool collision;

  /// Evaluation status of the edge.
  bool evaluated;
};

}  // namespace gls

#endif  // GLS_DATASTRUCTURES_GRAPH_HPP_
