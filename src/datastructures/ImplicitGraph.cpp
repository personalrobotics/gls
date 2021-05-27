/* Authors: Matt Schmittle */

#include "gls/datastructures/ImplicitGraph.hpp"

namespace gls {
namespace datastructures {

VertexProperties ImplicitGraph::operator[](IVertex key){
    return mVertices[key];
}
EdgeProperties ImplicitGraph::operator[](IEdge key){
    return mEdges[key];
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

void ImplicitGraph::addVertex(vertex_descriptor vi){
    if(mVertices.find(vi) == mVertices.end()){

        // Add to map
        mVertices[vi] = VertexProperties();
        //TODO add vi content to properties
    }
}

void ImplicitGraph::addEdge(vertex_descriptor v1, vertex_descriptor v2){
    IEdge ei = std::pair<IVertex, IVertex>{v1, v2}; 
    if(mEdges.find(ei) == mEdges.end()){

        // Add to map
        mEdges[ei] = EdgeProperties();
        //TODO add ei content to properties
    }
}

// Bosst similar API Functions
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

IVertex source(IEdge e, ImplicitGraph g) {
    return e.first;
}
IVertex target(IEdge e, ImplicitGraph g) {
    return e.second;
}

// wish I could return iterator, but can't because vector is made in function
std::vector<IVertex> adjacent_vertices(IVertex u, ImplicitGraph& g) {
  // Add vertex if not in graph
  g.addVertex(u); 

  // Generate neighbors
  std::vector<IVertex> neighbors = g.neighbors(u);

  // Update internal graph
  for(IVertexIter it = neighbors.begin(); it != neighbors.end(); ++it) {
      g.addVertex(*it); // Add vertex to map if not already in
      g.addEdge(u, *it); // Add edge to map if not already in
  }
  return neighbors;
}

} // namespace datastructures
} // namespace gls
