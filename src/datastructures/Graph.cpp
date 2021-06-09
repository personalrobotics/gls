/* Authors: Aditya Vamsikrishna Mandalika */

#include "gls/datastructures/Graph.hpp"

namespace gls {
namespace datastructures {

// ============================================================================
Vertex& Graph::addVertex(Vertex v, StatePtr state){
    if(mImplicit){
        mImplicitGraph.addVertex(v.mImplicitVertex, state);
    }
    else{
        mExplicitGraph[v.mExplicitVertex].setState(state);
    }
    mVertices.push_back(v);
    return mVertices.back();
}

// ============================================================================
std::pair<Edge&, bool> Graph::addEdge(Vertex v1, Vertex v2){
    bool exists;
    Edge newEdge;
    if (mImplicit){
        //TODO (schmittle) this never gets run and we don't have the edge length so set to 0
        std::pair<IEdge, bool> raw_edge = mImplicitGraph.addEdge(v1.mImplicitVertex, v2.mImplicitVertex, 0, 0);
        newEdge = Edge(v1, v2, true);
        newEdge.mImplicitEdge = raw_edge.first;
        exists = raw_edge.second;
    }
    else{
        newEdge = Edge(v1,v2, false);
        std::pair<EEdge, bool> raw_edge  = add_edge(v1.mExplicitVertex, v2.mExplicitVertex, mExplicitGraph); 
        newEdge.mExplicitEdge = raw_edge.first;
        exists = raw_edge.second;
    }
    EdgeHash hash;
    mEdges.push_back(newEdge);
    mEdgesLookup[hash(std::pair<Vertex, Vertex>{v1, v2})] = mEdges.back();
    return std::pair<Edge&, bool>{mEdges.back(), exists};
}

// ============================================================================
void Graph::setImplicit(ImplicitGraph g){
    mImplicitGraph = g;
    mImplicit = true;
}

// ============================================================================
VertexProperties & Graph::operator[](Vertex key) {
    if (mImplicit){
        return mImplicitGraph[key.mImplicitVertex];
    }
    return mExplicitGraph[key.mExplicitVertex];
}

// ============================================================================
EdgeProperties & Graph::operator[](Edge key) {
    if (mImplicit){
        return mImplicitGraph[key.mImplicitEdge];
    }
    return mExplicitGraph[key.mExplicitEdge];
}

// ============================================================================
std::pair<Edge, bool> Graph::operator[](std::string hash) {
    if(mEdgesLookup.find(hash) != mEdgesLookup.end()){
        return std::pair<Edge, bool>{mEdgesLookup[hash], true};
    }
    Edge* e = new Edge(); // blank edge
    return std::pair<Edge, bool>{*e, false};
}

// ============================================================================
std::pair<VertexIter, VertexIter> Graph::vertices(){
    return std::pair<VertexIter,VertexIter>{mVertices.begin(), mVertices.end()};
}

// ============================================================================
std::pair<EdgeIter, EdgeIter> Graph::edges(){
    return std::pair<EdgeIter, EdgeIter>{mEdges.begin(), mEdges.end()};
}

// ============================================================================
std::pair<NeighborIter, NeighborIter> Graph::adjacents(Vertex u){
    mAdjacents.clear();
    EdgeHash hash;
    if(mImplicit){
        std::vector<std::tuple<IVertex, bool, bool>> raw_adjs = gls::datastructures::adjacent_vertices(u.mImplicitVertex, mImplicitGraph);
        mAdjacents.reserve(raw_adjs.size());
        for (std::tuple<IVertex, bool, bool> vi : raw_adjs){
            mAdjacents.push_back(Vertex(std::get<0>(vi)));

            // Update mVertices, mEdges if we expanded internally
            if(!std::get<1>(vi)){ // vertex didn't exist before now 
                Vertex v(std::get<0>(vi)); 
                mVertices.push_back(v);
            }
            if(!std::get<2>(vi)){ // edge didn't exist before now 
                Edge newEdge = Edge(u, mAdjacents.back(), true); // I THINK this is okay 
                mEdges.push_back(newEdge);
                mEdgesLookup[hash(std::pair<Vertex, Vertex>{u, mAdjacents.back()})] = mEdges.back();
            }
        }
    }
    else{
        ENeighborIter ni, ni_end;
        for (boost::tie(ni, ni_end) = boost::adjacent_vertices(u.mExplicitVertex, mExplicitGraph); ni != ni_end; ++ni) {
            mAdjacents.push_back(Vertex(*ni));
        }
    }
    return std::pair<NeighborIter, NeighborIter>{mAdjacents.begin(), mAdjacents.end()};
}

// ============================================================================
const std::map<std::string, Edge>& Graph::getLookup(){ // never use this function
    return mEdgesLookup;
}

// ============================================================================
void Graph::updateExplicit(){
    // Update mVertices to match
    EVertexIter vi, vi_end;
    for (boost::tie(vi, vi_end) = boost::vertices(mExplicitGraph); vi != vi_end; ++vi) {
        Vertex v(*vi);
        mVertices.push_back(v);
    }
    // Update mEdges to match
    EdgeHash hash;
    EEdgeIter ei, ei_end;
    for (boost::tie(ei, ei_end) = boost::edges(mExplicitGraph); ei != ei_end; ++ei) {
        EVertex u = boost::source(*ei, mExplicitGraph);
        EVertex v = boost::target(*ei, mExplicitGraph);

        Edge newEdge = Edge(*ei); // I THINK this is okay
        mEdges.push_back(newEdge);
        mEdgesLookup[hash(std::pair<Vertex, Vertex>{Vertex(u), Vertex(v)})] = mEdges.back();
    }
}

// ============================================================================
std::pair<VertexIter, VertexIter> vertices(Graph& g){
    return g.vertices();
}

// ============================================================================
std::pair<EdgeIter, EdgeIter> edges(Graph& g){
    return g.edges();
}

// ============================================================================
std::pair<NeighborIter, NeighborIter> adjacent_vertices(Vertex u, Graph& g){
    return g.adjacents(u);
}

// ============================================================================
Vertex addVertex(Graph& g, StatePtr state){
    IVertexHash hash;
    if(g.mImplicit){
        Vertex vert = Vertex(hash(g.mImplicitGraph.fit2Lat(state)));
        return g.addVertex(vert, state);
    }
    else{
        EVertex evert = add_vertex(g.mExplicitGraph);
        return g.addVertex(Vertex(evert), state);
    }
}

// ============================================================================
std::pair<Edge&, bool> addEdge(Vertex v1, Vertex v2, Graph& g){
    return g.addEdge(v1, v2);
}

// ============================================================================
std::pair<Edge, bool> edge(Vertex v1, Vertex v2, Graph& g){
    EdgeHash hash;
    std::string hashed_edge = hash(std::pair<Vertex, Vertex>{v1, v2});
    std::string reverse_hashed_edge = hash(std::pair<Vertex, Vertex>{v2, v1});
    /*
    std::map<std::string, Edge> lookup = g.getLookup();
    if(lookup.find(hashed_edge) != lookup.end()){
        return std::pair<Edge, bool>{lookup[hashed_edge], true};
    }
    if(!g.mImplicit && lookup.find(reverse_hashed_edge) != lookup.end()){ // TODO (schmittle) check reverse 4 implicit?
        return std::pair<Edge, bool>{lookup[reverse_hashed_edge], true};
    }
    Edge* e = new Edge(); // blank edge
    return std::pair<Edge, bool>{*e, false};
    */
    /// This is faster than above
    std::pair<Edge, bool> edge_lookup = g[hashed_edge];
    if(!g.mImplicit && !edge_lookup.second){
        edge_lookup = g[reverse_hashed_edge];
    }
    return edge_lookup;
}

// ============================================================================
Vertex source(Edge e, Graph& g){
    if(e.isImplicit()){
        return e.first;
    }
    else{
        return boost::source(e.mExplicitEdge, g.mExplicitGraph);
    }
}

// ============================================================================
Vertex target(Edge e, Graph& g){
    if(e.isImplicit()){
        return e.second;
    }
    else{
        return boost::target(e.mExplicitEdge, g.mExplicitGraph);
    }
}

// ============================================================================
void clear_vertex(Vertex v, Graph& g){
    if (!g.mImplicit){
        clear_vertex(v.mExplicitVertex, g.mExplicitGraph);
    }
    else{
        // TODO
    }
}

// ============================================================================
void remove_vertex(Vertex v, Graph& g){
    if (!g.mImplicit){
        remove_vertex(v.mExplicitVertex, g.mExplicitGraph);
    }
    else{
        // TODO
    }
}

} // namespace datastructures
} // namespace gls
