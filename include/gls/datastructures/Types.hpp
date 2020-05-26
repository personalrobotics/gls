/* Authors: Aditya Vamsikrishna Mandalika */

#ifndef GLS_DATASTRUCTURES_TYPES_HPP_
#define GLS_DATASTRUCTURES_TYPES_HPP_

// Boost headers
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>
#include <boost/property_map/dynamic_property_map.hpp>

namespace gls {

/// Basic Boost graph.
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS>
    BasicGraph;

/// Shared pointer to a BasicGraph.
typedef std::shared_ptr<BasicGraph> BasicGraphPtr;

/// Shared pointer to a const BasicGraph.
typedef std::shared_ptr<const BasicGraph> ConstBasicGraphPtr;

/// Boost Vertex.
typedef boost::graph_traits<BasicGraph>::vertex_descriptor Vertex;

/// Shared pointer to a Vertex.
typedef std::shared_ptr<Vertex> VertexPtr;

/// Shared pointer to a const Vertex.
typedef std::shared_ptr<const Vertex> ConstVertexPtr;

/// Path represented by a series of vertices.
typedef std::vector<Vertex> Path;

/// Shared pointer to a Path.
typedef std::shared_ptr<Path> PathPtr;

/// Shared pointer to a const Path.
typedef std::shared_ptr<const Path> ConstPathPtr;

/// Boost edge
typedef boost::graph_traits<BasicGraph>::edge_descriptor Edge;

/// Shared pointer to an Edge.
typedef std::shared_ptr<Edge> EdgePtr;

/// Shared pointer to a const Edge.
typedef std::shared_ptr<const Edge> ConstEdgePtr;

/// Flags to determine collision status of vertex/edge.
enum CollisionStatus { Collision, Free };

/// Flags to determine visitation of a vertex.
enum VisitStatus { NotVisited, Visited };

/// Flags to determine the evaluation status of a vertex/edge.
enum EvaluationStatus { NotEvaluated, Evaluated };

/// Flags indicating the tree validity status.
enum TreeValidityStatus { Valid, NotValid };

/// Flags indicating if the planner has solved the problem.
enum PlannerStatus { Solved, NotSolved };

}  // namespace gls

#endif  // GLS_DATASTRUCTURES_TYPES_HPP_
