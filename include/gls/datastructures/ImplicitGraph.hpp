#ifndef GLS_DATASTRUCTURES_IMPLICITGRAPH_HPP_
#define GLS_DATASTRUCTURES_IMPLICITGRAPH_HPP_

// STL headers
#include <vector>
#include <string>

// OMPL header
#include <ompl/base/spaces/RealVectorStateSpace.h>

// GLS headers
#include "gls/datastructures/State.hpp"
#include "gls/datastructures/Types.hpp"
#include "gls/datastructures/Properties.hpp"

namespace gls {
namespace datastructures {

class ImplicitGraph {
    public:
        typedef std::string vertex_descriptor;
        typedef std::pair<vertex_descriptor, vertex_descriptor> edge_descriptor;
        typedef std::unordered_map<std::string, std::pair<edge_descriptor, EdgeProperties>> edge_hash_table;

        typedef std::vector<vertex_descriptor>::iterator vertex_iterator;
        typedef std::vector<std::tuple<vertex_descriptor, VertexProperties, double, int>>::iterator neighbor_iterator;
        typedef std::vector<edge_descriptor>::iterator edge_iterator;
        typedef std::function<StatePtr(StatePtr)> fdiscretize;
        typedef std::function<std::vector<std::tuple<vertex_descriptor, VertexProperties, double, int>>(vertex_descriptor, VertexProperties)> fneighbors;
        typedef std::function<std::string(StatePtr)> fhash;

        ImplicitGraph(){};
        ImplicitGraph(fdiscretize discretizefunc, 
                fneighbors neighborfunc, 
                fneighbors parentfunc, 
                fhash hashfunc):
            fit2Lat(discretizefunc), 
            neighbors(neighborfunc),
            parents(parentfunc),
            hash(hashfunc){};

        VertexProperties & operator[](vertex_descriptor key);
        EdgeProperties & operator[](edge_descriptor key); 

        std::unordered_map<vertex_descriptor, VertexProperties> get_vmap() const;
        edge_hash_table get_emap() const;
        std::size_t num_vertices() const;
        std::size_t num_edges() const;

        fdiscretize fit2Lat; // member passed by constructor
        fneighbors neighbors; // member passed by constructor
        fneighbors parents; // member passed by constructor
        fhash hash; // Implicit hash function 

        bool addVertex(vertex_descriptor vi, StatePtr state);
        bool addAdjVertex(vertex_descriptor vi, StatePtr state); // add vertex w/o fit
        std::pair<edge_descriptor, bool> addEdge(vertex_descriptor v1, vertex_descriptor v2, double length, int motprimID);

    private:
        std::unordered_map<vertex_descriptor, VertexProperties> mVertices;
        edge_hash_table mEdges;
        std::unordered_map<std::string, edge_descriptor> mEdgeHashTable;
};

// For map TODO (Schmittle) this should be user defined since it has a specific statepspace
/*
struct IVertexHash{
    std::string operator()(StatePtr k) {
        double* values = k->getOMPLState()->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        return std::to_string((int)round(values[0])) + 
            std::to_string((int)round(values[1])) + 
            std::to_string((int)round(values[2])) + 
            std::to_string((int)round(values[4])) + 
            std::to_string((int)round(values[5]));
        //debug + std::to_string((int)round(values[6])); // Ignore block angle
    }
};
*/

typedef ImplicitGraph::vertex_descriptor IVertex;
typedef ImplicitGraph::edge_descriptor IEdge; 
typedef std::shared_ptr<IVertex> IVertexPtr;
typedef std::shared_ptr<IEdge> IEdgePtr;
typedef ImplicitGraph::edge_hash_table IEdgeTable; 

typedef ImplicitGraph::fneighbors NeighborFunc;
typedef ImplicitGraph::fdiscretize DiscFunc;
typedef ImplicitGraph::fhash HashFunc;
typedef std::function<std::vector<StatePtr>(StatePtr, StatePtr, int, bool)> InterpolateFunc;

/// vertex/edge iterator
typedef ImplicitGraph::vertex_iterator IVertexIter;
typedef ImplicitGraph::neighbor_iterator INeighborIter;
typedef ImplicitGraph::edge_iterator IEdgeIter;

// Functions
std::size_t num_vertices(const ImplicitGraph& g);
std::vector<IVertex> vertices(const ImplicitGraph& g);
IVertex source(const IEdge& e, const ImplicitGraph& g);
IVertex target(const IEdge& e, const ImplicitGraph& g);

// Functions
std::vector<IEdge> edges(const ImplicitGraph& g);
std::size_t num_edges(const ImplicitGraph& g);


// Functions
std::vector<std::tuple<IVertex, bool, bool>> parent_vertices(const IVertex& u, ImplicitGraph& g);
std::vector<std::tuple<IVertex, bool, bool>> adjacent_vertices(const IVertex& u, ImplicitGraph& g);

// Extra Functions
void clear_vertex(Vertex v, ImplicitGraph g); // Does nothing
void remove_vertex(Vertex v, ImplicitGraph g); // Does nothing

} // namespace datastructures
} // namespace gls

#endif // GLS_IMPLICITDATASTRUCTURES_GRAPH_HPP_
