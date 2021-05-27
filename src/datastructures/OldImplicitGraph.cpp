/* Authors: Matt Schmittle */

#include "gls/datastructures/ImplicitGraph.hpp"

namespace gls {
namespace datastructures {

IVertex ImplicitGraph::operator[](IVertexPtr){

}

// VertexListGraph valid expressions
vertices_size_type num_vertices(const ImplicitGraph& g) {
    return g.n();
};

edges_size_type num_edges(const ImplicitGraph& g) {
    return g.n() == 1 ? 1:g.n();
};

std::pair<IVertexIter, IVertexIter> vertices(const ImplicitGraph& g) {
    return std::pair<IVertexIter, IVertexIter>(
            IVertexIter(0), // The first iterator position
            IVertexIter(num_vertices(g))); // The last iterator position
}
std::pair<IEdgeIter, IEdgeIter> edges(const ImplicitGraph& g) {
    return std::pair<IEdgeIter, IEdgeIter>(
            IEdgeIter(g, iterator_start()), 
            IEdgeIter(g, iterator_end())); 
}

IVertex source(IEdge e, ImplicitGraph g) {
    return e.first;
}
IVertex target(IEdge e, ImplicitGraph g) {
    return e.second;
}

std::pair<out_edge_iterator, out_edge_iterator>
out_edges(IVertex u, const ImplicitGraph& g) {
  return std::pair<out_edge_iterator, out_edge_iterator>(
    out_edge_iterator(g, u, iterator_start()),
    out_edge_iterator(g, u, iterator_end()) );
}

std::pair<adjacency_iterator, adjacency_iterator>
adjacent_vertices(IVertex u, const ImplicitGraph& g) {
  return std::pair<adjacency_iterator, adjacency_iterator>(
    adjacency_iterator(u, g, iterator_start()),
    adjacency_iterator(u, g, iterator_end()));
}

} // namespace datastructures
} // namespace gls
