#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <vector>
#include "src/graph.hpp"


TEST(Graph, should_return_direct_reachable_nodes) {
    
    mylib::Graph<int> graph;
    graph.add_edge(mylib::Edge<int>(1, 0, true, 50.));
    graph.add_edge(mylib::Edge<int>(1, 2, true, 17.));
    graph.add_edge(mylib::Edge<int>(0, 1, true, 10.));

    std::set<std::pair<int, double>> nodes;
    auto next = graph.get_next_reachable(1);
    if (next) {
        for (auto node: next.get()) {
            nodes.insert(std::make_pair(node.first, node.second.get_distance()));
        }
    }
    ASSERT_THAT(nodes, testing::UnorderedElementsAre(testing::Pair(0, 50.), testing::Pair(2,17.)));
}


/** 
 * Test path length of std::vector<Node>
 */

TEST(Graph, should_return_path_length) {
    mylib::Graph<int> graph;
    graph.add_edge(mylib::Edge<int>(1, 0, true, 50.));
    graph.add_edge(mylib::Edge<int>(1, 2, true, 17.));
    graph.add_edge(mylib::Edge<int>(0, 1, true, 10.));
    
    ASSERT_EQ(graph.get_path_length({0, 1, 2}), 27);
}

/** 
 * Test path length of std::vector<Node>
 */

TEST(Graph, should_return_correct_path_length_on_no_path) {
    mylib::Graph<int> graph;
    graph.add_edge(mylib::Edge<int>(1, 0, true, 50.));
    graph.add_edge(mylib::Edge<int>(1, 2, true, 17.));
    graph.add_edge(mylib::Edge<int>(0, 1, true, 10.));
    
    ASSERT_EQ(graph.get_path_length({0, 2, 1}), -1);
}


/**
 * Test Dijkstra
 */

TEST(Graph, should_dijkstra_compute_the_shortest_path) {
  mylib::Graph<int> graph;
  // Fill the graph https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm

  graph.add_symmetric_edge(mylib::Edge<int>(1, 2, true, 7.));
  graph.add_symmetric_edge(mylib::Edge<int>(1, 3, true, 9.));
  graph.add_symmetric_edge(mylib::Edge<int>(1, 6, true, 14.));
  
  graph.add_symmetric_edge(mylib::Edge<int>(2, 1, true, 7.));
  graph.add_symmetric_edge(mylib::Edge<int>(2, 3, true, 10.));
  graph.add_symmetric_edge(mylib::Edge<int>(2, 4, true, 15.));

  graph.add_symmetric_edge(mylib::Edge<int>(3, 6, true, 2.));
  graph.add_symmetric_edge(mylib::Edge<int>(3, 4, true, 6.));

  graph.add_symmetric_edge(mylib::Edge<int>(4, 5, true, 6.));
  graph.add_symmetric_edge(mylib::Edge<int>(5, 6, true, 9.));

  ASSERT_THAT(graph.dijkstra(1, 5), testing::ElementsAre(1, 3, 6, 5));

}

// TODO: Test shortest path index from a vector of path
