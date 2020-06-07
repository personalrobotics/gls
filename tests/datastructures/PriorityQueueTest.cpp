#include <gtest/gtest.h>

#include "gls/datastructures/Graph.hpp"
#include "gls/datastructures/PriorityQueue.hpp"

class PriorityQueueTest : public ::testing::Test {
 protected:
  /// Setup the test fixture.
  void SetUp() override {  // Do nothing.
  }
};

// ==============================================================================
TEST_F(PriorityQueueTest, CreateEmptyNamedQueue) {
  auto searchQueue = gls::PriorityQueue<gls::FValueComparator>();
  EXPECT_EQ(searchQueue.getName(), "PriorityQueue");
  EXPECT_TRUE(searchQueue.isEmpty());

  auto namedSearchQueue = gls::PriorityQueue<gls::FValueComparator>("name");
  EXPECT_EQ(namedSearchQueue.getName(), "name");
  EXPECT_TRUE(namedSearchQueue.isEmpty());
}

// ==============================================================================
TEST_F(PriorityQueueTest, AddNode) {
  auto searchQueue = gls::PriorityQueue<gls::FValueComparator>();
  EXPECT_TRUE(searchQueue.isEmpty());
  searchQueue.addNode(gls::Node());
  EXPECT_FALSE(searchQueue.isEmpty());
}

// ==============================================================================
TEST_F(PriorityQueueTest, HasNode) {
  auto searchQueue = gls::PriorityQueue<gls::FValueComparator>();
  gls::Node n;
  n.costToCome = 1;
  searchQueue.addNode(n);
  EXPECT_TRUE(searchQueue.hasNode(n));
}

// ==============================================================================
TEST_F(PriorityQueueTest, PopNode) {
  auto searchQueue = gls::PriorityQueue<gls::FValueComparator>();
  EXPECT_TRUE(searchQueue.isEmpty());
  searchQueue.addNode(gls::Node());
  EXPECT_FALSE(searchQueue.isEmpty());
  searchQueue.popTopNode();
  EXPECT_TRUE(searchQueue.isEmpty());
}

// ==============================================================================
TEST_F(PriorityQueueTest, CompareFValue) {
  auto searchQueue = gls::PriorityQueue<gls::FValueComparator>();

  // Key: 4
  gls::Node n1;
  n1.costToCome = 1;
  n1.heuristic = 3;

  // Key: 3
  gls::Node n2;
  n2.costToCome = 2;
  n2.heuristic = 1;

  // Key: 2
  gls::Node n3;
  n3.costToCome = 2;
  n3.heuristic = 0;

  searchQueue.addNode(n1);
  searchQueue.addNode(n2);
  searchQueue.addNode(n3);

  EXPECT_FALSE(searchQueue.isEmpty());

  // Check the order in the queue.
  EXPECT_EQ(searchQueue.getTopNodeTotalCost(), 2);
  searchQueue.removeNode(n3);
  EXPECT_EQ(searchQueue.getTopNodeTotalCost(), 3);
  searchQueue.removeNode(n2);
  EXPECT_EQ(searchQueue.getTopNodeTotalCost(), 4);
  searchQueue.removeNode(n1);

  EXPECT_TRUE(searchQueue.isEmpty());
}

// ==============================================================================
TEST_F(PriorityQueueTest, CompareGValue) {
  auto searchQueue = gls::PriorityQueue<gls::GValueComparator>();

  // Key: 1
  gls::Node n1;
  n1.costToCome = 1;
  n1.heuristic = 3;

  // Key: 4
  gls::Node n2;
  n2.costToCome = 4;
  n2.heuristic = 1;

  // Key: 2
  gls::Node n3;
  n3.costToCome = 2;
  n3.heuristic = 0;

  searchQueue.addNode(n1);
  searchQueue.addNode(n2);
  searchQueue.addNode(n3);

  EXPECT_FALSE(searchQueue.isEmpty());

  // Check the order in the queue.
  EXPECT_EQ(searchQueue.getTopNodeCostToCome(), 1);
  searchQueue.removeNode(n1);
  EXPECT_EQ(searchQueue.getTopNodeCostToCome(), 2);
  searchQueue.removeNode(n3);
  EXPECT_EQ(searchQueue.getTopNodeCostToCome(), 4);
  searchQueue.removeNode(n2);

  EXPECT_TRUE(searchQueue.isEmpty());
}

// ==============================================================================
TEST_F(PriorityQueueTest, TieBreaking) {
  auto fqueue = gls::PriorityQueue<gls::FValueComparator>();
  auto gqueue = gls::PriorityQueue<gls::GValueComparator>();

  // [g, f] = [1,2]
  gls::Node n1;
  n1.id = 1;
  n1.costToCome = 1;
  n1.heuristic = 2;

  // [g, f] = [1,2]
  gls::Node n2;
  n2.id = 2;
  n2.costToCome = 1;
  n2.heuristic = 2;

  gqueue.addNode(n1);
  gqueue.addNode(n2);

  fqueue.addNode(n1);
  fqueue.addNode(n2);

  EXPECT_FALSE(fqueue.isEmpty());
  EXPECT_FALSE(gqueue.isEmpty());

  // Check the order in the queue.
  gls::Node topNode = fqueue.popTopNode();
  EXPECT_EQ(topNode.id, 1u);
  gls::Node bottomNode = fqueue.popTopNode();
  EXPECT_EQ(bottomNode.id, 2u);
  EXPECT_TRUE(fqueue.isEmpty());

  topNode = gqueue.popTopNode();
  EXPECT_EQ(topNode.id, 1u);
  bottomNode = gqueue.popTopNode();
  EXPECT_EQ(bottomNode.id, 2u);
  EXPECT_TRUE(gqueue.isEmpty());
}

// ==============================================================================
TEST_F(PriorityQueueTest, UpdateKeyValue) {
  auto searchQueue = gls::PriorityQueue<gls::GValueComparator>();

  // Key: 1
  gls::Node n1;
  n1.costToCome = 1;

  // Key: 4
  gls::Node n2;
  n2.costToCome = 4;

  // Key: 2
  gls::Node n3;
  n3.costToCome = 2;

  searchQueue.addNode(n1);
  searchQueue.addNode(n2);
  searchQueue.addNode(n3);
  EXPECT_EQ(searchQueue.getSize(), 3u);

  // Check the order in the queue.
  gls::Node r1 = searchQueue.popTopNode();
  EXPECT_EQ(r1.costToCome, 1);
  gls::Node r2 = searchQueue.popTopNode();
  EXPECT_EQ(r2.costToCome, 2);
  gls::Node r3 = searchQueue.popTopNode();
  EXPECT_EQ(r3.costToCome, 4);
  EXPECT_TRUE(searchQueue.isEmpty());

  // Update the key for n1
  searchQueue.addNode(n1);
  searchQueue.addNode(n2);
  searchQueue.addNode(n3);
  EXPECT_EQ(searchQueue.getSize(), 3u);

  searchQueue.updateNodeCostToCome(n1, 5);
  r1 = searchQueue.popTopNode();
  EXPECT_EQ(r1.costToCome, 2);
  r2 = searchQueue.popTopNode();
  EXPECT_EQ(r2.costToCome, 4);
  r3 = searchQueue.popTopNode();
  EXPECT_EQ(r3.costToCome, 5);
  EXPECT_TRUE(searchQueue.isEmpty());
}
