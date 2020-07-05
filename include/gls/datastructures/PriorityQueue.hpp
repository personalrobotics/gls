// Author: Aditya Vamsikrishna Mandalika
// Date: 6th June 2020

#ifndef GLS_DATASTRUCTURES_PRIORITYQUEUE_HPP_
#define GLS_DATASTRUCTURES_PRIORITYQUEUE_HPP_

#include <set>

#include "gls/datastructures/Graph.hpp"

namespace gls {

/// \brief Priority Queue of node IDs sorted by comparator function.
class PriorityQueue {
 public:
  /// \brief The function signature of the sorting function for the queue.
  using PriorityQueueComparator =
      std::function<bool(const std::size_t&, const std::size_t&)>;
  /// \brief The underlying vertex queue.
  using NodeQueue = std::set<std::size_t, PriorityQueueComparator>;
  /// \brief An iterator into the vertex queue set.
  using PriorityQueueIterator = NodeQueue::iterator;

  /// Contructor.
  /// \param[in] name Name of the queue.
  explicit PriorityQueue(const Graph& graph,
                         const std::string& name = "PriorityQueue")
      : mGraph(graph),
        mName(name),
        mNodeQueue([this](const std::size_t& lhs, const std::size_t& rhs) {
          return queueComparison(lhs, rhs);
        }) {
    // Do nothing.
  }

  virtual ~PriorityQueue() = default;

  /// Returns a reference to the top node of the queue.
  inline std::size_t getTopNode() const { return *(mNodeQueue.begin()); }

  /// Returns the top node ID after popping it from the queue..
  inline std::size_t popTopNode() {
    std::size_t id = *(mNodeQueue.begin());
    mNodeQueue.erase(mNodeQueue.begin());
    return id;
  };

  /// Returns the cost-to-come corresponding to the top node in the queue.
  inline double getTopNodeCostToCome() const {
    return mGraph[*(mNodeQueue.begin())].costToCome;
  };

  /// Returns the heuristic corresponding to the top node in the queue.
  inline double getTopNodeHeuristic() const {
    return mGraph[*(mNodeQueue.begin())].heuristic;
  };

  /// Returns the f-value corresponding to the top node in the queue.
  inline double getTopNodeTotalCost() const {
    return mGraph[*(mNodeQueue.begin())].totalCost();
  };

  /// Updates the cost-to-come of the top node in the queue and resorts.
  // TODO (avk): This seems to be more expensive than find + remove + replace.
  // inline void updateNodeCostToCome(const std::size_t& id, const double& cost)
  // {
  //   auto nodeHandle = mNodeQueue.extract(id);
  //   mGraph[nodeHandle.value()].costToCome = cost;
  //   mNodeQueue.insert(std::move(nodeHandle));
  // };

  /// Adds a node to the queue.
  inline void addNode(const std::size_t& id) { mNodeQueue.emplace(id); };

  /// Removes a node from the queue.
  inline void removeNode(const std::size_t& id) {
    auto iter = mNodeQueue.find(id);
    if (iter != mNodeQueue.end()) {
      mNodeQueue.erase(iter);
    } else {
      // TODO(avk): Replace this with a warning message.
      std::cout << "Cannot find node" << std::endl;
    }
  }

  /// Returns true if the node exists in the queue.
  inline bool hasNode(const std::size_t& id) const {
    return (mNodeQueue.find(id) != mNodeQueue.end());
  }

  /// Returns the size of the queue.
  inline std::size_t getSize() const { return mNodeQueue.size(); }

  /// Return true if the queue is empty.
  inline bool isEmpty() const { return mNodeQueue.empty(); };

  /// Returns the name of the queue.
  inline const std::string& getName() const { return mName; }

 protected:
  /// A lexicographical comparison function for a pair of node IDs.
  /// As a default, it sorts the node by their IDs in ascending order.
  bool virtual queueComparison(const std::size_t& lhs,
                               const std::size_t& rhs) const {
    return lhs < rhs;
  }

  /// A const reference to the graph whose nodes feature in the queue.
  /// The graph is owned by the planner and is only modifiable by the planner.
  const Graph& mGraph;

 private:
  /// The name of the queue.
  const std::string& mName;

  /// The underlying queue, wraps std::set of Node IDs.
  NodeQueue mNodeQueue;
};

/// A queue of nodes sorted according to g-values.
class GValueQueue final : public PriorityQueue {
 public:
  /// \brief Constructor.
  /// Construct a search queue. It must be setup before use.
  GValueQueue(const Graph& graph, std::string name = "GValueQueue")
      : PriorityQueue(graph, name) {}

  /// \brief Destructor.
  ~GValueQueue() = default;

  /// Compares nodes according to gvalues.
  bool queueComparison(const std::size_t& lhs,
                       const std::size_t& rhs) const override {
    const double& left = mGraph[lhs].costToCome;
    const double& right = mGraph[rhs].costToCome;
    if (left != right) {
      return left < right;
    }
    return PriorityQueue::queueComparison(lhs, rhs);
  }
};

/// A queue of nodes sorted according to g-values.
class FValueQueue final : public PriorityQueue {
 public:
  /// \brief Constructor.
  /// Construct a search queue. It must be setup before use.
  FValueQueue(const Graph& graph, std::string name = "FValueQueue")
      : PriorityQueue(graph, name) {}

  /// \brief Destructor.
  ~FValueQueue() = default;

  /// Compares nodes according to gvalues.
  bool queueComparison(const std::size_t& lhs,
                       const std::size_t& rhs) const override {
    const double& left = mGraph[lhs].totalCost();
    const double& right = mGraph[rhs].totalCost();
    if (left != right) {
      return left < right;
    }
    return PriorityQueue::queueComparison(lhs, rhs);
  }
};

}  // namespace gls

#endif  // GLS_DATASTRUCTURES_PRIORITYQUEUE_HPP_
