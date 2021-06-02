/* Authors: Aditya Vamsikrishna Mandalika */

#include "gls/datastructures/Graph.hpp"

namespace gls {
namespace datastructures {

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
void Graph::incrementVertices(){
   mVertexNum++; 
}

// ============================================================================
std::pair<VertexIter, VertexIter> Graph::vertices(){
    mVertices.clear();
    if(mImplicit){
        std::vector<IVertex> raw_verts = gls::datastructures::vertices(mImplicitGraph);
        mVertices.reserve(raw_verts.size());
        for (IVertex vi : raw_verts){
            mVertices.push_back(Vertex(vi));
        }
    }
    else{
        EVertexIter vi, vi_end;
        for (boost::tie(vi, vi_end) = boost::vertices(mExplicitGraph); vi != vi_end; ++vi) {
            mVertices.push_back(Vertex(*vi));
        }
    }
    return std::pair<VertexIter,VertexIter>{mVertices.begin(), mVertices.end()};
}

// ============================================================================
std::pair<EdgeIter, EdgeIter> Graph::edges(){
    mEdges.clear();
    if(mImplicit){
        std::vector<IEdge> raw_edges = gls::datastructures::edges(mImplicitGraph);
        mEdges.reserve(raw_edges.size());
        for (IEdge ei : raw_edges){
            mEdges.push_back(Edge(ei));
        }
    }
    else{
        EEdgeIter ei, ei_end;
        for (boost::tie(ei, ei_end) = boost::edges(mExplicitGraph); ei != ei_end; ++ei) {
            mEdges.push_back(Edge(*ei));
        }
    }
    return std::pair<EdgeIter, EdgeIter>{mEdges.begin(), mEdges.end()};
}

// ============================================================================
std::pair<NeighborIter, NeighborIter> Graph::adjacents(Vertex u){
    mAdjacents.clear();
    if(mImplicit){
        std::vector<IVertex> raw_adjs = gls::datastructures::adjacent_vertices(u.mImplicitVertex, mImplicitGraph);
        mAdjacents.reserve(raw_adjs.size());
        for (IVertex vi : raw_adjs){
            mAdjacents.push_back(Vertex(vi));
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
std::pair<VertexIter, VertexIter> vertices(Graph g){
    return g.vertices();
}

// ============================================================================
std::pair<EdgeIter, EdgeIter> edges(Graph g){
    return g.edges();
}

// ============================================================================
std::pair<NeighborIter, NeighborIter> adjacent_vertices(Vertex u, Graph g){
    return g.adjacents(u);
}

// ============================================================================
Vertex addVertex(Graph g, StatePtr state){
    if(g.mImplicit){
        Vertex vert = Vertex(std::to_string(g.mVertexNum));
        g.mImplicitGraph.addVertex(vert.mImplicitVertex, state);
        g.incrementVertices();
        return vert;
    }
    else{
        EVertex evert = add_vertex(g.mExplicitGraph);
        g.mExplicitGraph[evert].setState(state);
        return Vertex(evert);
    }
}

// ============================================================================
std::pair<Edge, bool> addEdge(Vertex v1, Vertex v2, Graph g){
    if (g.mImplicit){
        std::pair<IEdge, bool> raw_edge = g.mImplicitGraph.addEdge(v1.mImplicitVertex, v2.mImplicitVertex);
        Edge newEdge = Edge(v1, v2);
        newEdge.mImplicitEdge = raw_edge.first;// TODO confirm with updated api
        return std::pair<Edge, bool>{newEdge, raw_edge.second};
    }
    else{
        Edge newEdge = Edge(v1,v2);
        std::pair<EEdge, bool> raw_edge  = add_edge(v1.mExplicitVertex, v2.mExplicitVertex, g.mExplicitGraph); 
        newEdge.mExplicitEdge = raw_edge.first;
        return std::pair<Edge, bool>{newEdge, raw_edge.second};
    }
}

// ============================================================================
std::pair<Edge, bool> edge(Vertex v1, Vertex v2, Graph g){
    return addEdge(v1, v2, g);
}

// ============================================================================
Vertex source(Edge e, Graph g){
    return e.first;
}

// ============================================================================
Vertex target(Edge e, Graph g){
    return e.second;
}

// ============================================================================
void clear_vertex(Vertex v, Graph g){
    if (!g.mImplicit){
        clear_vertex(v.mExplicitVertex, g.mExplicitGraph);
    }
    else{
        // TODO
    }
}

// ============================================================================
void remove_vertex(Vertex v, Graph g){
    if (!g.mImplicit){
        remove_vertex(v.mExplicitVertex, g.mExplicitGraph);
    }
    else{
        // TODO
    }
}

} // namespace datastructures
} // namespace gls
