// Author: Aditya Vamsikrishna Mandalika
// Date: 6th June 2020

#ifndef GLS_DATASTRUCTURES_PRIORITYQUEUE_HPP_
#define GLS_DATASTRUCTURES_PRIORITYQUEUE_HPP_

#include <set>

#include "gls/datastructures/Graph.hpp"

namespace gls {

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

template <class comparator>
class PriorityQueue {
 public:
  explicit PriorityQueue(const std::string& name = "PriorityQueue")
      : mName(name) {
    // Do nothing.
  }
  inline const Node& getTopNode() const { return *mNodeQueue.begin(); }
  inline Node popTopNode() {
    Node node = *(mNodeQueue.begin());
    mNodeQueue.erase(mNodeQueue.begin());
    return node;
  };
  inline double getTopNodeCostToCome() const {
    return mNodeQueue.begin()->costToCome;
  };
  inline double getTopNodeHeuristic() const {
    return mNodeQueue.begin()->heuristic;
  };
  inline double getTopNodeTotalCost() const {
    return mNodeQueue.begin()->totalCost();
  };
  inline void updateNodeCostToCome(Node node, const double cost) {
    auto nodeHandle = mNodeQueue.extract(node);
    nodeHandle.value().costToCome = cost;
    mNodeQueue.insert(std::move(nodeHandle));
  };
  inline void addNode(const Node& node) { mNodeQueue.emplace(node); };
  inline void removeNode(const Node& node) {
    auto iter = mNodeQueue.find(node);
    if (iter != mNodeQueue.end()) {
      mNodeQueue.erase(iter);
    } else {
      std::cout << "Cannot find node" << std::endl;
    }
  }
  inline bool hasNode(const Node& node) const {
    return (mNodeQueue.find(node) != mNodeQueue.end());
  }
  inline std::size_t getSize() const { return mNodeQueue.size(); }
  inline bool isEmpty() const { return mNodeQueue.empty(); };
  inline const std::string& getName() const { return mName; }

 private:
  std::set<Node, comparator> mNodeQueue;
  const std::string& mName;
};

}  // namespace gls

#endif  // GLS_DATASTRUCTURES_SEARCHQUEUE_HPP_
