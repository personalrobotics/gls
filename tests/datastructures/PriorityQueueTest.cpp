#include <gtest/gtest.h>

#include "gls/datastructures/Graph.hpp"
#include "gls/datastructures/PriorityQueue.hpp"

class PriorityQueueTest : public ::testing::Test {
 protected:
  /// Setup the test fixture.
  void SetUp() override{};
};

// ==============================================================================
TEST_F(PriorityQueueTest, CreateEmptyNamedQueue) {
  gls::Graph g;
  auto searchQueue = gls::PriorityQueue(g);
  EXPECT_EQ(searchQueue.getName(), "PriorityQueue");
  EXPECT_TRUE(searchQueue.isEmpty());

  auto namedSearchQueue = gls::PriorityQueue(g, "name");
  EXPECT_EQ(namedSearchQueue.getName(), "name");
  EXPECT_TRUE(namedSearchQueue.isEmpty());
}

// ==============================================================================
TEST_F(PriorityQueueTest, AddNode) {
  gls::Graph g;
  auto searchQueue = gls::PriorityQueue(g);
  auto s = boost::add_vertex(g);
  EXPECT_TRUE(searchQueue.isEmpty());
  searchQueue.addNode(s);
  EXPECT_FALSE(searchQueue.isEmpty());
}

// ==============================================================================
TEST_F(PriorityQueueTest, HasNode) {
  gls::Graph g;
  auto searchQueue = gls::PriorityQueue(g);
  auto s = boost::add_vertex(g);
  searchQueue.addNode(s);
  EXPECT_TRUE(searchQueue.hasNode(s));
}

// ==============================================================================
TEST_F(PriorityQueueTest, PopNode) {
  gls::Graph g;
  auto s = boost::add_vertex(g);
  auto searchQueue = gls::PriorityQueue(g);
  EXPECT_TRUE(searchQueue.isEmpty());
  searchQueue.addNode(s);
  EXPECT_FALSE(searchQueue.isEmpty());
  searchQueue.popTopNode();
  EXPECT_TRUE(searchQueue.isEmpty());
}

// ==============================================================================
TEST_F(PriorityQueueTest, CompareID) {
  gls::Graph g;
  auto s = boost::add_vertex(g);
  auto t = boost::add_vertex(g);
  auto searchQueue = gls::PriorityQueue(g);

  searchQueue.addNode(s);
  searchQueue.addNode(t);

  EXPECT_FALSE(searchQueue.isEmpty());

  // Check the order in the queue.
  EXPECT_EQ(searchQueue.getTopNode(), s);
  searchQueue.removeNode(s);
  EXPECT_EQ(searchQueue.getTopNode(), t);
  searchQueue.removeNode(t);

  EXPECT_TRUE(searchQueue.isEmpty());
}

// ==============================================================================
TEST_F(PriorityQueueTest, CompareGValue) {
  gls::Graph g;
  auto s = boost::add_vertex(g);
  auto v = boost::add_vertex(g);
  auto t = boost::add_vertex(g);
  auto searchQueue = gls::GValueQueue(g);

  // Key: 1
  g[s].costToCome = 1;
  g[s].heuristic = 3;

  // Key: 4
  g[v].costToCome = 4;
  g[v].heuristic = 1;

  // Key: 2
  g[t].costToCome = 2;
  g[t].heuristic = 0;

  searchQueue.addNode(s);
  searchQueue.addNode(v);
  searchQueue.addNode(t);

  EXPECT_FALSE(searchQueue.isEmpty());

  // Check the order in the queue.
  EXPECT_EQ(searchQueue.getTopNodeCostToCome(), 1);
  EXPECT_EQ(searchQueue.getTopNode(), s);
  searchQueue.removeNode(s);

  EXPECT_EQ(searchQueue.getTopNodeCostToCome(), 2);
  EXPECT_EQ(searchQueue.getTopNode(), t);
  searchQueue.removeNode(t);

  EXPECT_EQ(searchQueue.getTopNodeCostToCome(), 4);
  EXPECT_EQ(searchQueue.getTopNode(), v);
  searchQueue.removeNode(v);

  EXPECT_TRUE(searchQueue.isEmpty());
}

// ==============================================================================
TEST_F(PriorityQueueTest, CompareFValue) {
  gls::Graph g;
  auto s = boost::add_vertex(g);
  auto v = boost::add_vertex(g);
  auto t = boost::add_vertex(g);
  auto searchQueue = gls::FValueQueue(g);

  // Key: 4
  g[s].costToCome = 1;
  g[s].heuristic = 3;

  // Key: 5
  g[v].costToCome = 4;
  g[v].heuristic = 1;

  // Key: 2
  g[t].costToCome = 2;
  g[t].heuristic = 0;

  searchQueue.addNode(s);
  searchQueue.addNode(v);
  searchQueue.addNode(t);

  EXPECT_FALSE(searchQueue.isEmpty());

  // Check the order in the queue.
  EXPECT_EQ(searchQueue.getTopNodeTotalCost(), 2);
  EXPECT_EQ(searchQueue.getTopNode(), t);
  searchQueue.removeNode(t);

  EXPECT_EQ(searchQueue.getTopNodeTotalCost(), 4);
  EXPECT_EQ(searchQueue.getTopNode(), s);
  searchQueue.removeNode(s);

  EXPECT_EQ(searchQueue.getTopNodeTotalCost(), 5);
  EXPECT_EQ(searchQueue.getTopNode(), v);
  searchQueue.removeNode(v);

  EXPECT_TRUE(searchQueue.isEmpty());
}

// ==============================================================================
TEST_F(PriorityQueueTest, TieBreaking) {
  gls::Graph g;
  auto s = boost::add_vertex(g);
  auto t = boost::add_vertex(g);

  auto fqueue = gls::FValueQueue(g);
  auto gqueue = gls::GValueQueue(g);

  g[s].costToCome = 1;
  g[s].heuristic = 2;

  g[t].costToCome = 1;
  g[t].heuristic = 2;

  gqueue.addNode(s);
  gqueue.addNode(t);

  fqueue.addNode(s);
  fqueue.addNode(t);

  EXPECT_FALSE(fqueue.isEmpty());
  EXPECT_FALSE(gqueue.isEmpty());

  // Check the order in the queue.
  EXPECT_EQ(gqueue.getTopNode(), s);
  EXPECT_EQ(fqueue.getTopNode(), s);
}
