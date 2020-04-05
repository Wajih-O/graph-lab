#include <boost/optional.hpp>
#include <gtest/gtest.h>

#include "test_edge.hpp"
#include "test_graph.hpp"

#include "test_hex.hpp"

boost::optional<int> f(int x){
  if (x>0) {
    return x;
  }
  return {};
}


TEST(OptionalPositiv, shouldcomparecorrectly) {
  ASSERT_EQ(4, f(4).value());
}
