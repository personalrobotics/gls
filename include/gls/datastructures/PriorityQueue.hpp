// Author: Aditya Vamsikrishna Mandalika
// Date: 6th June 2020

#ifndef GLS_DATASTRUCTURES_PRIORITYQUEUE_HPP_
#define GLS_DATASTRUCTURES_PRIORITYQUEUE_HPP_

#include <set>

#include "gls/datastructures/Graph.hpp"

namespace gls {

/// Compares nodes based on their F-Value. Breaks ties by ID.
struct FValueComparator {
  inline bool operator()(const Node& left, const Node& right) {
    const double& leftTotalCost = left.totalCost();
    const double& rightTotalCost = right.totalCost();
    if (leftTotalCost != rightTotalCost) {
      return leftTotalCost < rightTotalCost;
    }
    return left.id < right.id;
  }
};

/// Compares nodes based on their G-Value. Breaks ties by ID.
struct GValueComparator {
  inline bool operator()(const Node& left, const Node& right) {
    const double& leftCost = left.costToCome;
    const double& rightCost = right.costToCome;
    if (leftCost != rightCost) {
      return leftCost < rightCost;
    }
    return left.id < right.id;
  }
};

/// \brief Priority Queue templated by the custom comparator function.
/// The class wraps an std::set and provides utility functions to
/// query the top node, the key, and update the nodes as required.
template <class comparator>
class PriorityQueue {
 public:
  /// Contructor.
  /// \param[in] name Name of the queue.
  explicit PriorityQueue(const std::string& name = "PriorityQueue")
      : mName(name) {
    // Do nothing.
  }

  /// Returns a reference to the top node of the queue.
  inline const Node& getTopNode() const { return *mNodeQueue.begin(); }

  /// Returns a copy of the top node after removing it from the queue.
  inline Node popTopNode() {
    Node node = *(mNodeQueue.begin());
    mNodeQueue.erase(mNodeQueue.begin());
    return node;
  };

  /// Returns the cost-to-come corresponding to the top node in the queue.
  inline double getTopNodeCostToCome() const {
    return mNodeQueue.begin()->costToCome;
  };

  /// Returns the heuristic corresponding to the top node in the queue.
  inline double getTopNodeHeuristic() const {
    return mNodeQueue.begin()->heuristic;
  };

  /// Returns the f-value corresponding to the top node in the queue.
  inline double getTopNodeTotalCost() const {
    return mNodeQueue.begin()->totalCost();
  };

  /// Updates the cost-to-come of the top node in the queue and resorts.
  inline void updateNodeCostToCome(Node node, const double cost) {
    auto nodeHandle = mNodeQueue.extract(node);
    nodeHandle.value().costToCome = cost;
    mNodeQueue.insert(std::move(nodeHandle));
  };

  /// Adds a node to the queue.
  inline void addNode(const Node& node) { mNodeQueue.emplace(node); };

  /// Removes a node from the queue.
  inline void removeNode(const Node& node) {
    auto iter = mNodeQueue.find(node);
    if (iter != mNodeQueue.end()) {
      mNodeQueue.erase(iter);
    } else {
      std::cout << "Cannot find node" << std::endl;
    }
  }

  /// Returns true if the node exists in the queue.
  inline bool hasNode(const Node& node) const {
    return (mNodeQueue.find(node) != mNodeQueue.end());
  }

  /// Returns the size of the queue.
  inline std::size_t getSize() const { return mNodeQueue.size(); }

  /// Return true if the queue is empty.
  inline bool isEmpty() const { return mNodeQueue.empty(); };

  /// Returns the name of the queue.
  inline const std::string& getName() const { return mName; }

 private:
  /// The underlying queue, wraps std::set.
  std::set<Node, comparator> mNodeQueue;

  /// The name of the queue.
  const std::string& mName;
};

}  // namespace gls

#endif  // GLS_DATASTRUCTURES_PRIORITYQUEUE_HPP_
