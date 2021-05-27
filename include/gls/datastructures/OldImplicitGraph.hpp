/* Authors: Matt Schmittle */

#ifndef GLS_DATASTRUCTURES_IMPLICITGRAPH_HPP_
#define GLS_DATASTRUCTURES_IMPLICITGRAPH_HPP_

// STL headers
#include <set>
#include <vector>

// Boost headers
#include <boost/iterator/counting_iterator.hpp>

// GLS headers
#include "gls/datastructures/Graph.hpp"
#include "gls/datastructures/State.hpp"
#include "gls/datastructures/Types.hpp"

namespace gls {
namespace datastructures {

// Forward declarations
class implicit_iterator;
class implicit_edge_iterator;
class implicit_adjacency_iterator;

struct implicit_traversal_catetory:
  virtual public boost::bidirectional_graph_tag,
  virtual public boost::adjacency_graph_tag,
  virtual public boost::vertex_list_graph_tag,
  virtual public boost::edge_list_graph_tag
    {};

class ImplicitGraph {
    public:
        typedef std::size_t vertex_descriptor;
        typedef boost::undirected_tag directed_category;
        typedef boost::disallow_parallel_edge_tag edge_parallel_category;
        typedef implicit_traversal_catetory traversal_category;
        typedef std::pair<vertex_descriptor, vertex_descriptor> edge_descriptor;

        typedef implicit_iterator out_edge_iterator;

        typedef boost::counting_iterator<vertex_descriptor> vertex_iterator;
        typedef std::size_t vertices_size_type;
        typedef implicit_edge_iterator edge_iterator;
        typedef std::size_t edges_size_type;
        typedef implicit_adjacency_iterator adjacency_iterator;

        ImplicitGraph(std::size_t n):m_n(n) {};
        vertex_descriptor operator[](std::shared_ptr<vertex_descriptor>); // TODO
        edge_descriptor operator[](std::shared_ptr<edge_descriptor>); // TODO
        std::size_t n() const {return m_n;}

    private:
        std::size_t m_n;
};

struct iterator_position {};
struct iterator_start:virtual public iterator_position {};
struct iterator_end:virtual public iterator_position {};

typedef boost::graph_traits<ImplicitGraph>::vertex_descriptor IVertex;
typedef boost::graph_traits<ImplicitGraph>::edge_descriptor IEdge; 
typedef std::shared_ptr<IVertex> IVertexPtr;
typedef std::shared_ptr<IEdge> IEdgePtr;

/// Boost vertex/edge iterator
typedef boost::graph_traits<ImplicitGraph>::vertex_iterator IVertexIter;
typedef boost::graph_traits<ImplicitGraph>::edge_iterator IEdgeIter;
typedef boost::graph_traits<ImplicitGraph>::vertices_size_type vertices_size_type;
typedef boost::graph_traits<ImplicitGraph>::edges_size_type edges_size_type;
typedef boost::graph_traits<ImplicitGraph>::out_edge_iterator out_edge_iterator;
typedef boost::graph_traits<ImplicitGraph>::adjacency_iterator adjacency_iterator;

// Incident iterator
class implicit_iterator:public boost::iterator_adaptor <
    implicit_iterator,
    boost::counting_iterator<std::size_t>,
    IEdge,
    boost::use_default,
    IEdge > {
public:
  implicit_iterator():
    implicit_iterator::iterator_adaptor_(0),m_n(0),m_u(0) {};
  explicit implicit_iterator(const ImplicitGraph& g,
                                       IVertex u,
                                       iterator_start):
    implicit_iterator::iterator_adaptor_(0),
    m_n(g.n()),m_u(u) {};
  explicit implicit_iterator(const ImplicitGraph& g,
                                       IVertex u,
                                       iterator_end):
    // A graph with one vertex only has a single self-loop.  A graph with
    // two vertices has a single edge between them.  All other graphs have
    // two edges per vertex.
    implicit_iterator::iterator_adaptor_(g.n() > 2 ? 2:1),
    m_n(g.n()),m_u(u) {};

private:
  friend class boost::iterator_core_access;

  IEdge dereference() const {
    static const int ring_offset[] = {1, -1};
    IVertex v;

    std::size_t p = *this->base_reference();
    if (m_u == 0 && p == 1)
      v = m_n-1; // Vertex n-1 precedes vertex 0.
    else
      v = (m_u+ring_offset[p]) % m_n;
    return IEdge(m_u, v);
  }

  std::size_t m_n; // Size of the graph
  Vertex m_u; // Vertex whose out edges are iterated
};

// Functions
void add_vertex(ImplicitGraph g); // TODO
vertices_size_type num_vertices(const ImplicitGraph& g);
std::pair<IVertexIter, IVertexIter> vertices(const ImplicitGraph& g);
IVertex source(IEdge e, ImplicitGraph g);
IVertex target(IEdge e, ImplicitGraph g);
std::pair<out_edge_iterator, out_edge_iterator>out_edges(IVertex u, const ImplicitGraph& g);


// Edge iterator
class implicit_edge_iterator:public boost::iterator_adaptor<
  implicit_edge_iterator,
  IVertexIter,
  IEdge,
  boost::use_default,
  IEdge > {
public:
  implicit_edge_iterator():
    implicit_edge_iterator::iterator_adaptor_(0),m_g(NULL) {};
  explicit implicit_edge_iterator(const ImplicitGraph& g, iterator_start):
    implicit_edge_iterator::iterator_adaptor_(vertices(g).first),m_g(&g) {};
  explicit implicit_edge_iterator(const ImplicitGraph& g, iterator_end):
    implicit_edge_iterator::iterator_adaptor_(
      // Size 2 graphs have a single edge connecting the two vertices.
      g.n() == 2 ? ++(vertices(g).first) : vertices(g).second ),
    m_g(&g) {};

private:
  friend class boost::iterator_core_access;

  IEdge dereference() const {
    // The first element in the incident edge list of the current vertex.
    return *(out_edges(*this->base_reference(), *m_g).first);
  }

  // The graph being iterated over
  const ImplicitGraph *m_g;
};

// Functions
std::pair<IEdgeIter, IEdgeIter> edges(const ImplicitGraph& g);
std::pair<IEdge, bool> edge(Vertex u, Vertex v, ImplicitGraph g); // TODO
edges_size_type num_edges(const ImplicitGraph& g);

// Adjacency iterator
class implicit_adjacency_iterator:public boost::adjacency_iterator_generator<
  ImplicitGraph,
  IVertex,
  out_edge_iterator>::type {
  // The parent class is an iterator_adpator that turns an iterator over
  // out edges into an iterator over adjacent vertices.
  typedef boost::adjacency_iterator_generator<
    ImplicitGraph,
    IVertex,
    out_edge_iterator>::type parent_class;
public:
  implicit_adjacency_iterator() {};
  implicit_adjacency_iterator(IVertex u,
                          const ImplicitGraph& g,
                          iterator_start):
    parent_class(out_edge_iterator(g, u, iterator_start()), &g) {};
  implicit_adjacency_iterator(IVertex u,
                          const ImplicitGraph& g,
                          iterator_end):
    parent_class(out_edge_iterator(g, u, iterator_end()), &g) {};
};

// Functions
std::pair<adjacency_iterator, adjacency_iterator>
adjacent_vertices(IVertex u, const ImplicitGraph& g);

// Extra Functions
void clear_vertex(Vertex v, ImplicitGraph g); // Does nothing
void remove_vertex(Vertex v, ImplicitGraph g); // Does nothing

} // namespace datastructures
} // namespace gls

#endif // GLS_IMPLICITDATASTRUCTURES_GRAPH_HPP_
