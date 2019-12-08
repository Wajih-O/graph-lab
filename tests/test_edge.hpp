#include <gtest/gtest.h>
#include "src/edge.hpp"

TEST(Edge, should_edge_reverse_ends) {
    auto edge = mylib::Edge<int>(1, 2, 0.5, true);
    auto reversed = edge.sym_reverse();
    ASSERT_EQ(reversed.from(), edge.to());
    ASSERT_EQ(reversed.to(), edge.from());
    ASSERT_EQ(edge.get_value(), reversed.get_value());
}