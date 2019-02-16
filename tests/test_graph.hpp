#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <vector>
#include "src/graph.hpp"


TEST(Graph, should_return_direct_reachable_nodes) {
    mylib::Graph<int> graph;
    graph.add_edge(mylib::Edge<int>(1, 0, 50.));
    graph.add_edge(mylib::Edge<int>(1, 2, 17.));
    graph.add_edge(mylib::Edge<int>(0, 1, 10.));

    std::set<std::pair<int, double>> nodes;
    auto next = graph.neighbors(1);
    if (next) {
        for (auto node: next.get()) {
            nodes.insert(std::make_pair(node.first, node.second.get_value()));
        }
    }
    ASSERT_THAT(nodes, testing::UnorderedElementsAre(testing::Pair(0, 50.), testing::Pair(2,17.)));
}


/**
 * Test path length of std::vector<Node>
 */

TEST(Graph, should_return_path_length) {
    mylib::Graph<int> graph;
    graph.add_edge(mylib::Edge<int>(1, 0, 50.));
    graph.add_edge(mylib::Edge<int>(1, 2, 17.));
    graph.add_edge(mylib::Edge<int>(0, 1, 10.));

    ASSERT_EQ(graph.get_path_length({0, 1, 2}), 27);
}

/**
 * Test path length of std::vector<Node>
 */

TEST(Graph, should_return_correct_path_length_on_no_path) {
    mylib::Graph<int> graph;
    graph.add_edge(mylib::Edge<int>(1, 0, 50.));
    graph.add_edge(mylib::Edge<int>(1, 2, 17.));
    graph.add_edge(mylib::Edge<int>(0, 1, 10.));

    ASSERT_EQ(graph.get_path_length({0, 2, 1}), -1);
}

/**
 * Test graph edges number (E)
 */

TEST(Graph, should_return_correct_edges_number) {
    mylib::Graph<int> graph;
    graph.add_edge(mylib::Edge<int>(1, 0, 50.));
    graph.add_edge(mylib::Edge<int>(1, 2, 17.));
    graph.add_edge(mylib::Edge<int>(0, 1, 10.));

    ASSERT_EQ(graph.E(), 3);
}

/**
 * Test graph nodes/vertices number (V)
 */

TEST(Graph, should_return_correct_vertices_number) {
    mylib::Graph<int> graph;
    graph.add_edge(mylib::Edge<int>(1, 0, 50.));
    graph.add_edge(mylib::Edge<int>(1, 2, 17.));
    graph.add_edge(mylib::Edge<int>(0, 1, 10.));

    ASSERT_EQ(graph.get_nodes_number(), 3);
}


/**
 * Test update node
 */

TEST(Graph, should_update_edge_value) {
    mylib::Graph<int> graph;
    graph.add_edge(mylib::Edge<int>(1, 0, 50.));
    ASSERT_EQ(graph.set_edge_value(1, 0, 3.), true);
    ASSERT_EQ(graph.get_edge_value(1, 0).get(), 3.);
}


/**
 * Test Dijkstra
 */

TEST(Graph, should_dijkstra_compute_the_shortest_path) {
  mylib::Graph<int> graph;
  // Fill the graph https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm

  graph.add_symmetric_edge(mylib::Edge<int>(1, 2, 7.));
  graph.add_symmetric_edge(mylib::Edge<int>(1, 3, 9.));
  graph.add_symmetric_edge(mylib::Edge<int>(1, 6, 14.));

  graph.add_symmetric_edge(mylib::Edge<int>(2, 1, 7.));
  graph.add_symmetric_edge(mylib::Edge<int>(2, 3, 10.));
  graph.add_symmetric_edge(mylib::Edge<int>(2, 4, 15.));

  graph.add_symmetric_edge(mylib::Edge<int>(3, 6, 2.));
  graph.add_symmetric_edge(mylib::Edge<int>(3, 4, 6.));

  graph.add_symmetric_edge(mylib::Edge<int>(4, 5, 6.));
  graph.add_symmetric_edge(mylib::Edge<int>(5, 6, 9.));

  ASSERT_THAT(graph.dijkstra(1, 5), testing::ElementsAre(1, 3, 6, 5));

}


// TODO: test random generation of graph given a set of nodes, values range and a density param


// TODO: Test shortest path index from a vector of path

// TODO Test edge filter
