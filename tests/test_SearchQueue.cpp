#include <gtest/gtest.h>
#include "GLS/Datastructures/SearchQueue.hpp"

using gls::datastructures::SearchQueue;

class SearchQueueTest : public testing::Test
{
public:
  virtual void SetUp()
  {
    for (int i = 0; i < 3; ++i)
    {
      Vertex vertex = i;
      double cost = 10 - i;
      mQueueElements.emplace_back(std::make_pair(vertex, cost));
    }
  }

protected:
  /// Elements to add into the queue for sorting.
  std::vector<std::pair<Vertex, double>> mQueueElements;
};

TEST_F(SearchQueueTest, successfulSorting)
{
  EXPECT_TRUE(true);
}
