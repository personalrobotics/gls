#include <gtest/gtest.h>

#include "gls/datastructures/Graph.hpp"
#include "gls/selector/Selector.hpp"

class SelectorTest : public ::testing::Test {
 protected:
  /// Setup the test fixture.
  void SetUp() override{};
};

// ==============================================================================
TEST_F(SelectorTest, CreateNamedSelector) {
  gls::Graph g;
  auto selector = gls::Selector(g, 0, 1);
  EXPECT_EQ(selector.getName(), "Selector");

  auto namedSelector = gls::Selector(g, 0, 1, "name");
  EXPECT_EQ(namedSelector.getName(), "name");
}

// ==============================================================================
TEST_F(SelectorTest, EdgeSelectionUnevaluatedPath) {
  gls::Graph g;
  auto s = boost::add_vertex(g);
  auto v = boost::add_vertex(g);
  auto t = boost::add_vertex(g);
  std::pair<gls::Edge, bool> e1 = boost::add_edge(s, v, g);
  std::pair<gls::Edge, bool> e2 = boost::add_edge(v, t, g);
  std::vector<std::size_t> path{t, v, s};
  g[e1.first].evaluated = false;
  g[e2.first].evaluated = false;

  auto selector = gls::Selector(g, 0, 1);
  auto edgesToEvaluate = selector.selectEdgeToEvaluate(path);
  EXPECT_FALSE(edgesToEvaluate.empty());
  EXPECT_EQ(edgesToEvaluate.size(), 1);
  EXPECT_EQ(source(edgesToEvaluate.front(), g), s);
  EXPECT_EQ(target(edgesToEvaluate.front(), g), v);
}
