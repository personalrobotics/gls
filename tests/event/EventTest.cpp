#include <gtest/gtest.h>

#include "gls/datastructures/Graph.hpp"
#include "gls/event/Event.hpp"

class EventTest : public ::testing::Test {
 protected:
  /// Setup the test fixture.
  void SetUp() override{};
};

// ==============================================================================
TEST_F(EventTest, CreateNamedEvent) {
  gls::Graph g;
  auto event = gls::Event(g, 0, 1);
  EXPECT_EQ(event.getName(), "Event");

  auto namedEvent = gls::Event(g, 0, 1, "name");
  EXPECT_EQ(namedEvent.getName(), "name");
}

// ==============================================================================
TEST_F(EventTest, TriggerWithGoal) {
  gls::Graph g;
  auto event = gls::Event(g, 0, 1);
  EXPECT_TRUE(1);
  EXPECT_FALSE(0);
}
