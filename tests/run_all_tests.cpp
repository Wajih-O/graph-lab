#include <boost/optional.hpp>
#include <gtest/gtest.h>

#include "tests/test_edge.hpp"
#include "tests/test_graph.hpp"

boost::optional<int> f(int x){
  if (x>0) {
    return x;
  }
  return {};
}


TEST(OptionalPositiv, shouldcomparecorrectly) {
  ASSERT_EQ(4, f(4).value());
}
