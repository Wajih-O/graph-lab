#include <gtest/gtest.h>
#include "src/edge.hpp"

TEST(Edge, should_edge_reverse_ends) {
    auto edge = mylib::Edge<int>(1, 2, true, 0.5);
    auto reversed = edge.sym_reverse();
    ASSERT_EQ(reversed.from(), edge.to());
    ASSERT_EQ(reversed.to(), edge.from());
}