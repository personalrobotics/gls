/* Authors: Aditya Vamsikrishna Mandalika */

#ifndef GLS_DATASTRUCTURES_TYPES_HPP_
#define GLS_DATASTRUCTURES_TYPES_HPP_

// Boost headers
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>
#include <boost/property_map/dynamic_property_map.hpp>

namespace gls {
namespace datastructures {

/// Basic Boost graph
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS> BasicGraph;

/// Boost vertex
typedef boost::graph_traits<BasicGraph>::vertex_descriptor Vertex;

/// Boost vertex iterator
typedef boost::graph_traits<BasicGraph>::vertex_iterator VertexIter;

/// Boost edge
typedef boost::graph_traits<BasicGraph>::edge_descriptor Edge;

/// Boost edge iterator
typedef boost::graph_traits<BasicGraph>::edge_iterator EdgeIter;

/// Boost graph neighbor iterator
typedef boost::graph_traits<BasicGraph>::adjacency_iterator NeighborIter;

} // datastructures
} // gls

#endif // GLS_DATASTRUCTURES_TYPES_HPP_
