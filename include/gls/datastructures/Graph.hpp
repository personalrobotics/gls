/* Authors: Aditya Vamsikrishna Mandalika */

#ifndef GLS_DATASTRUCTURES_GRAPH_HPP_
#define GLS_DATASTRUCTURES_GRAPH_HPP_

// STL headers
#include <vector>
#include <string>

// Boost headers
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>
#include <boost/property_map/dynamic_property_map.hpp>

// GLS headers
#include "gls/datastructures/ImplicitGraph.hpp"

// TODO (avk): state and length are made public to accomodate the
// roadmapmanager which seems stupid. Change it if possible.

namespace gls {
namespace datastructures {

/// Undirected Explicit Boost graph using the properties just defined.
typedef boost::
    adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, VertexProperties, EdgeProperties>
        EGraph;

/// Boost vertex iterator
typedef boost::graph_traits<EGraph>::vertex_iterator EVertexIter;
typedef std::vector<Vertex>::iterator VertexIter;

/// Boost edge iterator
typedef boost::graph_traits<EGraph>::edge_iterator EEdgeIter;
typedef std::vector<Edge>::iterator EdgeIter;

/// Boost graph neighbor iterator
typedef boost::graph_traits<EGraph>::adjacency_iterator ENeighborIter;
typedef std::vector<Vertex>::iterator NeighborIter;

// Graph wrapper class around boost explicit and custom implicit graphs
class Graph {
    public:
        Graph():mVertexNum(0){};
        Graph(bool implicit):mImplicit(implicit), mVertexNum(0){};
        
        VertexProperties & operator[](Vertex key);
        EdgeProperties & operator[](Edge key);
        std::pair<Edge, bool> operator[](std::string hash);

        ImplicitGraph mImplicitGraph;
        EGraph mExplicitGraph;

        /// Which graph type it is
        bool isImplicit();

        /// Sets mImplicit to true of false
        void setGraphType(std::string name);

        /// Return number of vertices
        int numVertices();

        Vertex& addVertex(Vertex v, StatePtr state);
        std::pair<Edge&, bool> addEdge(Vertex v1, Vertex v2);
        void setImplicit(ImplicitGraph g);
        std::pair<VertexIter,VertexIter> vertices();
        std::pair<EdgeIter, EdgeIter> edges();

        std::pair<NeighborIter, NeighborIter> parents(Vertex u);
        std::pair<NeighborIter, NeighborIter> adjacents(Vertex u);

        void updateExplicit();
    private:
        bool mImplicit = false;
        int mVertexNum;

        // Needed for edges(Graph), vertices(Graph), & adjacent_vertices(Graph)
        std::vector<Vertex> mVertices;
        std::vector<Edge> mEdges;
        std::unordered_map<std::string, Edge> mEdgesLookup; // For O(1) lookup
        std::vector<Vertex> mAdjacents;
};

// Graph functions
std::pair<VertexIter, VertexIter> vertices(Graph& g);
std::pair<EdgeIter, EdgeIter> edges(Graph& g);
std::pair<NeighborIter, NeighborIter> parent_vertices(Vertex u, Graph& g);
std::pair<NeighborIter, NeighborIter> adjacent_vertices(Vertex u, Graph& g);
Vertex addVertex(Graph& g, StatePtr state);
std::pair<Edge&, bool> addEdge(Vertex v1, Vertex v2, Graph& g);
std::pair<Edge, bool> edge(Vertex v1, Vertex v2, Graph& g);
Vertex source(Edge e, Graph& g);
Vertex target(Edge e, Graph& g);

void clear_vertex(Vertex v, Graph& g);
void remove_vertex(Vertex v, Graph& g);

/// Shared pointer to Graph.
typedef std::shared_ptr<Graph> GraphPtr;

/// Shared pointer to const Graph.
typedef std::shared_ptr<const Graph> ConstGraphPtr;

/// Map each vertex to the underlying state [read from the graphml file]
typedef boost::property_map<EGraph, gls::datastructures::StatePtr VertexProperties::*>::type
    VPStateMap;

/// Map each edge to its length
typedef boost::property_map<EGraph, double EdgeProperties::*>::type EPLengthMap;


} // namespace datastructures
} // namespace gls

#endif // GLS_DATASTRUCTURES_GRAPH_HPP_
