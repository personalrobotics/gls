// Author: Aditya Vamsikrishna Mandalika
// Date: 5th July 2020

#ifndef GLS_SELECTOR_SELECTOR_HPP_
#define GLS_SELECTOR_SELECTOR_HPP_

#include <vector>

#include "gls/datastructures/Graph.hpp"

namespace gls {

/// \brief Base class for a selector that determines the edge to evaluate.
class Selector {
 public:
  /// Constructor.
  /// \param[in] graph The graph for any auxiliary information.
  Selector(const Graph& graph, const std::size_t sourceIndex,
           const std::size_t targetIndex, const std::string& name = "Selector")
      : mGraph(graph),
        mSourceIndex(sourceIndex),
        mTargetIndex(targetIndex),
        mName(name) {
    // Do nothing.
  }

  /// Destructor.
  virtual ~Selector() = default;

  /// The core function of the Selector. Returns an edge or a sequence of
  /// edges to evaluate. TODO(avk): Expose greediness.
  virtual std::vector<Edge> inline selectEdgeToEvaluate(
      const std::vector<std::size_t>& path) const {
    Edge e;
    bool edgeExists;

    // By default, returns the first edge unevaluated.
    // The path is assumed to be in reverse since we backtrack.
    std::vector<Edge> edgesToEvaluate;
    for (auto i = path.size() - 1; i > 0; --i) {
      boost::tie(e, edgeExists) = edge(path[i], path[i - 1], mGraph);
      if (!mGraph[e].evaluated) {
        edgesToEvaluate.push_back(e);
        return edgesToEvaluate;
      }
    }
    // Return an empty vector if there are no more edges to evaluate.
    // The control reaches here only if there is a problem in the planning.
    // TODO(avk): Replace this with a warning.
    std::cout << "No edges to evaluate" << std::endl;
    return std::vector<Edge>();
  }

  // Returns the name associated with the event.
  const inline std::string& getName() const { return mName; }

 protected:
  // The associated graph.
  const Graph& mGraph;

  // The source index.
  const std::size_t& mSourceIndex;

  // The target index.
  const std::size_t& mTargetIndex;

  // The name of the event.
  const std::string& mName;
};

}  // namespace gls

#endif  // GLS_SELECTOR_SELECTOR_HPP_
