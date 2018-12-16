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

/// Undirected Boost graph
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, VertexProperties, EdgeProperties> Graph;

} // datastructures
} // gls

#endif // GLS_DATASTRUCTURES_GRAPH_HPP_
