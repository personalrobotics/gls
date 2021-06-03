/* Authors: Matt Schmittle */

#include "gls/datastructures/ImplicitGraph.hpp"

namespace gls {
namespace datastructures {

VertexProperties & ImplicitGraph::operator[](IVertex key){
    return mVertices[key];
}
EdgeProperties & ImplicitGraph::operator[](IEdge key){
    return mEdges[mEdgeHashTable[key.first+key.second]];
}

std::map<IVertex, VertexProperties> ImplicitGraph::get_vmap() const{
    return mVertices;
}

std::map<IEdge, EdgeProperties> ImplicitGraph::get_emap() const{
    return mEdges;
}

std::size_t ImplicitGraph::num_vertices() const{
    return mVertices.size();
}

std::size_t ImplicitGraph::num_edges() const{
    return mEdges.size();
}

bool ImplicitGraph::addVertex(vertex_descriptor vi, StatePtr state){
    bool exists = true;
    if(mVertices.find(vi) == mVertices.end()){

        VertexProperties* vp = new VertexProperties();
        vp->setState(fit2Lat(state));

        // Add to map
        mVertices[vi] = *vp;
        exists = false;
    }
    return exists;
}

bool ImplicitGraph::addAdjVertex(vertex_descriptor vi, StatePtr state){
    bool exists = true;
    if(mVertices.find(vi) == mVertices.end()){

        VertexProperties* vp = new VertexProperties();
        vp->setState(state);

        // Add to map
        mVertices[vi] = *vp;
        exists = false;
    }
    return exists;
}

std::pair<IEdge, bool> ImplicitGraph::addEdge(vertex_descriptor v1, vertex_descriptor v2, double length){
    IEdge ei = {v1, v2}; // EdgeHash effectively
    std::string eh = v1 + v2; // EdgeHash effectively
    bool exists = true;
    if(mEdgeHashTable.find(eh) == mEdgeHashTable.end()){

        // Add to map
        mEdges[ei] = EdgeProperties();
        mEdges[ei].setEvaluationStatus(EvaluationStatus::NotEvaluated);
        mEdges[ei].setCollisionStatus(CollisionStatus::Free);
        mEdges[ei].setLength(length);
        mEdgeHashTable[eh] = ei;
        exists = false;
    }
    return std::pair<IEdge, bool>{ei, exists};
}

// Boost similar API Functions
std::size_t num_vertices(const ImplicitGraph& g) {
    return g.num_vertices();
}

std::size_t num_edges(const ImplicitGraph& g) {
    return g.num_edges();
}

std::vector<IVertex> vertices(const ImplicitGraph& g) {
    std::vector<IVertex> vertices;
    std::map<IVertex, VertexProperties> m = g.get_vmap();
    vertices.reserve(m.size());
    for(std::map<IVertex, VertexProperties>::iterator it = m.begin(); it != m.end(); ++it) {
          vertices.push_back(it->first);
    }
    return vertices;
}
std::vector<IEdge> edges(const ImplicitGraph& g) {
    std::vector<IEdge> edges;
    std::map<IEdge, EdgeProperties> m = g.get_emap();
    edges.reserve(m.size());
    for(std::map<IEdge, EdgeProperties>::iterator it = m.begin(); it != m.end(); ++it) {
          edges.push_back(it->first);
    }
    return edges;
}

IVertex source(const IEdge& e, const ImplicitGraph& g) {
    return e.first;
}
IVertex target(const IEdge& e, const ImplicitGraph& g) {
    return e.second;
}

// wish I could return iterator, but can't because vector is made in function
std::vector<std::tuple<IVertex, bool, bool>> adjacent_vertices(const IVertex& u, ImplicitGraph& g) {

  // Generate neighbors
  std::vector<std::tuple<IVertex, VertexProperties, double>> neighbors = g.neighbors(u, g[u]);

  std::vector<std::tuple<IVertex, bool, bool>> return_neighbors;

  // Update internal graph
  for(INeighborIter it = neighbors.begin(); it != neighbors.end(); ++it) {
      bool vexists = g.addAdjVertex(std::get<0>(*it), std::get<1>(*it).getState()); // Add vertex to map if not already in
      std::pair<IEdge, bool> edge_pair = g.addEdge(u, std::get<0>(*it), std::get<2>(*it)); // Add edge to map if not already in
      return_neighbors.push_back(std::tuple<IVertex, bool, bool>{std::get<0>(*it), vexists, edge_pair.second});
  }
  return return_neighbors;
}

} // namespace datastructures
} // namespace gls
