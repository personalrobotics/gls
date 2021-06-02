/* Authors: Aditya Vamsikrishna Mandalika */

#ifndef GLS_DATASTRUCTURES_TYPES_HPP_
#define GLS_DATASTRUCTURES_TYPES_HPP_

// Boost headers
#include <boost/graph/adjacency_list.hpp>

namespace gls {
namespace datastructures {

/// Basic Boost graph.
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS> BasicGraph;

/// Shared pointer to a BasicGraph.
typedef std::shared_ptr<BasicGraph> BasicGraphPtr;

/// Shared pointer to a const BasicGraph.
typedef std::shared_ptr<const BasicGraph> ConstBasicGraphPtr;

/// Boost vertex.
typedef boost::graph_traits<BasicGraph>::vertex_descriptor EVertex;

/// Boost edge
typedef boost::graph_traits<BasicGraph>::edge_descriptor EEdge;


} // namespace datastructures
} // namespace gls

#endif // GLS_DATASTRUCTURES_TYPES_HPP_
