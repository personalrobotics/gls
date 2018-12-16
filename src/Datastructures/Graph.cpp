/* Authors: Aditya Vamsikrishna Mandalika */

#include "GLS/Datastructures/Graph.hpp"
#include "GLS/Datastructures/Types.hpp"
// TODO (avk): Is this circular dependency avoidable?
// i.e. I depend on Types.hpp and Types.hpp depends on Graph.hpp.

namespace gls {
namespace datastructures {

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