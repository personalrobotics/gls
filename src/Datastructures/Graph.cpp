/* Authors: Aditya Vamsikrishna Mandalika */

#include "GLS/Datastructures/Graph.hpp"

namespace gls {
namespace graph {

struct VertexProperties
{
  /// Cost-to-Come
  double costToCome;

  /// Heuristic value
  double heuristic;

  /// Parent
  Vertex parent;

  /// Children
  std::vector<Vertex> children;
};

struct EdgeProperties
{
  /// The length of the edge using the space distance metric
  double length;

  /// Flag to check if edge is evaluated
  bool isEvaluated;
};

} // graph
} // gls