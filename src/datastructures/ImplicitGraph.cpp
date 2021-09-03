/* Authors: Matt Schmittle */

#include "gls/datastructures/ImplicitGraph.hpp"

namespace gls {
namespace datastructures {

VertexProperties & ImplicitGraph::operator[](IVertex key){
    return mVertices[key];
}
EdgeProperties & ImplicitGraph::operator[](IEdge key){
    return mEdges[key.first+key.second].second;
}

std::unordered_map<IVertex, VertexProperties> ImplicitGraph::get_vmap() const{
    return mVertices;
}

IEdgeTable ImplicitGraph::get_emap() const{
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
    if(mVertices.count(vi) == 0){

        VertexProperties* vp = new VertexProperties();
        vp->setState(fit2Lat(state));
        vp->setVisitStatus(VisitStatus::NotVisited);
        vp->setCollisionStatus(CollisionStatus::Free);
        vp->setCostToCome(std::numeric_limits<double>::max());
        vp->setHeuristic(std::numeric_limits<double>::max());

        // Add to map
        mVertices[vi] = *vp;
        exists = false;
    }
    return exists;
}

bool ImplicitGraph::addAdjVertex(vertex_descriptor vi, StatePtr state){
    bool exists = true;
    if(mVertices.count(vi) == 0){

        VertexProperties* vp = new VertexProperties();
        vp->setState(state);

        // Add to map
        mVertices[vi] = *vp;
        exists = false;
    }
    return exists;
}

std::pair<IEdge, bool> ImplicitGraph::addEdge(vertex_descriptor v1, vertex_descriptor v2, double length, int motprimID){
    IEdge ei = {v1, v2};
    std::string eh = v1 + v2; // EdgeHash effectively
    bool exists = true;
    if(mEdges.count(eh) == 0){

        // Add to map
        mEdges[eh].second = EdgeProperties();
        mEdges[eh].second.setEvaluationStatus(EvaluationStatus::NotEvaluated);
        mEdges[eh].second.setCollisionStatus(CollisionStatus::Free);
        mEdges[eh].second.setLength(length);
        mEdges[eh].second.setPrimID(motprimID);
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
    std::unordered_map<IVertex, VertexProperties> m = g.get_vmap();
    vertices.reserve(m.size());
    for(std::unordered_map<IVertex, VertexProperties>::iterator it = m.begin(); it != m.end(); ++it) {
          vertices.push_back(it->first);
    }
    return vertices;
}
std::vector<IEdge> edges(const ImplicitGraph& g) {
    std::vector<IEdge> edges;
    IEdgeTable m = g.get_emap();
    edges.reserve(m.size());
    for(IEdgeTable::iterator it = m.begin(); it != m.end(); ++it) {
          edges.push_back((it->second).first);
    }
    return edges;
}

IVertex source(const IEdge& e, const ImplicitGraph& g) {
    return e.first;
}
IVertex target(const IEdge& e, const ImplicitGraph& g) {
    return e.second;
}

std::vector<std::tuple<IVertex, bool, bool>> parent_vertices(const IVertex& u, ImplicitGraph& g) {

  // Generate neighbors: vertex_descriptor, VertexProperties, length, motprimID
  std::vector<std::tuple<IVertex, VertexProperties, double, int>> neighbors = g.parents(u, g[u]);

  std::vector<std::tuple<IVertex, bool, bool>> return_neighbors;

  // Update internal graph
  for(INeighborIter it = neighbors.begin(); it != neighbors.end(); ++it) {
      bool vexists = g.addAdjVertex(std::get<0>(*it), std::get<1>(*it).getState()); // Add vertex to map if not already in
      std::pair<IEdge, bool> edge_pair = g.addEdge(std::get<0>(*it), u, std::get<2>(*it), std::get<3>(*it)); // Add edge to map if not already in
      return_neighbors.push_back(std::tuple<IVertex, bool, bool>{std::get<0>(*it), vexists, edge_pair.second});
  }
  return return_neighbors;
}

// wish I could return iterator, but can't because vector is made in function
std::vector<std::tuple<IVertex, bool, bool>> adjacent_vertices(const IVertex& u, ImplicitGraph& g) {

  // Generate neighbors: vertex_descriptor, VertexProperties, length, motprimID
  std::vector<std::tuple<IVertex, VertexProperties, double, int>> neighbors = g.neighbors(u, g[u]);

  std::vector<std::tuple<IVertex, bool, bool>> return_neighbors;

  // Update internal graph
  for(INeighborIter it = neighbors.begin(); it != neighbors.end(); ++it) {
      bool vexists = g.addAdjVertex(std::get<0>(*it), std::get<1>(*it).getState()); // Add vertex to map if not already in
      std::pair<IEdge, bool> edge_pair = g.addEdge(u, std::get<0>(*it), std::get<2>(*it), std::get<3>(*it)); // Add edge to map if not already in
      return_neighbors.push_back(std::tuple<IVertex, bool, bool>{std::get<0>(*it), vexists, edge_pair.second});
  }
  return return_neighbors;
}

} // namespace datastructures
} // namespace gls
