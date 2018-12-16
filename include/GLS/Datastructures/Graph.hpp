/* Authors: Aditya Vamsikrishna Mandalika */

#ifndef GLS_GRAPH_GRAPH_HPP_
#define GLS_GRAPH_GRAPH_HPP_

// STL headers
#include <vector>

// Boost headers
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>
#include <boost/property_map/dynamic_property_map.hpp>

namespace gls {
namespace graph {

struct VertexProperties;
struct EdgeProperties;

/// Undirected Boost graph
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, VertexProperties, EdgeProperties> Graph;

/// Boost vertex
typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;

/// Boost vertex iterator
typedef boost::graph_traits<Graph>::vertex_iterator VertexIter;

/// Boost edge
typedef boost::graph_traits<Graph>::edge_descriptor Edge;

/// Boost edge iterator
typedef boost::graph_traits<Graph>::edge_iterator EdgeIter;

/// Boost graph neighbor iterator
typedef boost::graph_traits<Graph>::adjacency_iterator NeighborIter;

} // graph
} // gls
#endif // GLS_GRAPH_GRAPH_HPP_
