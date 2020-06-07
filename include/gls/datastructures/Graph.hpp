// Author: Aditya Vamsikrishna Mandalika
// Date: 6th June 2020

#ifndef GLS_DATASTRUCTURES_GRAPH_HPP_
#define GLS_DATASTRUCTURES_GRAPH_HPP_

namespace gls {

struct Node {
  std::size_t id;
  double costToCome;
  double heuristic;
  bool collision;
  bool expanded;
  bool repair;

  constexpr double totalCost() const { return costToCome + heuristic; };
};

}  // namespace gls

#endif  // GLS_DATASTRUCTURES_GRAPH_HPP_
