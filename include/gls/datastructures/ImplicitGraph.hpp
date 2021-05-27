/* Authors: Matt Schmittle */

#ifndef GLS_DATASTRUCTURES_IMPLICITGRAPH_HPP_
#define GLS_DATASTRUCTURES_IMPLICITGRAPH_HPP_

// STL headers
#include <set>
#include <vector>
#include <string>

// GLS headers
#include "gls/datastructures/Graph.hpp"
#include "gls/datastructures/State.hpp"
#include "gls/datastructures/Types.hpp"

namespace gls {
namespace datastructures {

class ImplicitGraph {
    public:
        typedef std::string vertex_descriptor;
        typedef std::pair<vertex_descriptor, vertex_descriptor> edge_descriptor;

        typedef std::vector<vertex_descriptor>::iterator vertex_iterator;
        typedef std::vector<edge_descriptor>::iterator edge_iterator;
        typedef std::function<std::vector<vertex_descriptor>(vertex_descriptor)> fneighbors;

        ImplicitGraph(){};
        ImplicitGraph(fneighbors neighborfunc):neighbors(neighborfunc){};

        VertexProperties operator[](vertex_descriptor key);
        EdgeProperties operator[](edge_descriptor key); 

        std::map<vertex_descriptor, VertexProperties> get_vmap() const;
        std::map<edge_descriptor, EdgeProperties> get_emap() const;
        std::size_t num_vertices() const;
        std::size_t num_edges() const;

        fneighbors neighbors; // member passed by constructor

        void addVertex(vertex_descriptor vi);
        void addEdge(vertex_descriptor v1, vertex_descriptor v2);

    private:
        std::map<vertex_descriptor, VertexProperties> mVertices;
        std::map<edge_descriptor, EdgeProperties> mEdges;
};

typedef ImplicitGraph::vertex_descriptor IVertex;
typedef ImplicitGraph::edge_descriptor IEdge; 
typedef std::shared_ptr<IVertex> IVertexPtr;
typedef std::shared_ptr<IEdge> IEdgePtr;

typedef ImplicitGraph::fneighbors NeighborFunc;

/// vertex/edge iterator
typedef ImplicitGraph::vertex_iterator IVertexIter;
typedef ImplicitGraph::edge_iterator IEdgeIter;

// Functions
void add_vertex(ImplicitGraph g); // TODO
std::size_t num_vertices(const ImplicitGraph& g);
std::vector<IVertex> vertices(const ImplicitGraph& g);
IVertex source(IEdge e, ImplicitGraph g);
IVertex target(IEdge e, ImplicitGraph g);

// Functions
std::vector<IEdge> edges(const ImplicitGraph& g);
std::pair<IEdge, bool> edge(Vertex u, Vertex v, ImplicitGraph g); // TODO
std::size_t num_edges(const ImplicitGraph& g);


// Functions
std::vector<IVertex> adjacent_vertices(IVertex u, ImplicitGraph& g);

// Extra Functions
void clear_vertex(Vertex v, ImplicitGraph g); // Does nothing
void remove_vertex(Vertex v, ImplicitGraph g); // Does nothing

} // namespace datastructures
} // namespace gls

#endif // GLS_IMPLICITDATASTRUCTURES_GRAPH_HPP_
