#include "src/hex.hpp"
#include <gtest/gtest.h>

TEST(HexBoardTest, should_build_a_simple_hex_board_properly) {
  auto hex_board = mylib::HexBoard<int>(2, 2);
  ASSERT_TRUE(hex_board.check());
}

TEST(HexBoardTest, should_return_true_on_within_the_cell_board) {
  auto hex_board = mylib::HexBoard<int>(2, 2);
  ASSERT_TRUE(hex_board.within_the_board(std::make_pair(1, 1)));
}


TEST(HexBoardTest, should_return_false_on_within_the_cell_board) {
  auto hex_board = mylib::HexBoard<int>(2, 2);
  ASSERT_FALSE(hex_board.within_the_board(std::make_pair(2, 1)));
}


TEST(HexBoardTest, should_treat_edges_cases_properly) {
  auto hex_board = mylib::HexBoard<int>(3, 3);
  ASSERT_EQ(hex_board.all_neighbors(std::make_pair(0, 0)).size(), 2);
  ASSERT_EQ(hex_board.all_neighbors(std::make_pair(1, 1)).size(), 6);
  ASSERT_EQ(hex_board.all_neighbors(std::make_pair(0, 2)).size(), 3);
  ASSERT_EQ(hex_board.all_neighbors(std::make_pair(2, 2)).size(), 2);
}


TEST(HexBoardTest, should_build_graph_properly_for_the_first_player) {
  auto hex_board = mylib::HexBoard<int>(4, 4, 1); // builds one player config.
  ASSERT_TRUE(hex_board.assign_cell_to_player(std::make_pair(0, 0), 0));
  ASSERT_TRUE(hex_board.assign_cell_to_player(std::make_pair(0, 1), 0));
  ASSERT_TRUE(hex_board.assign_cell_to_player(std::make_pair(0, 2), 0));

  auto player_graph = hex_board.get_player_graph(0);
  ASSERT_EQ(player_graph.get()->get_edges_number(), 4);

  ASSERT_EQ(
      player_graph.get()->get_path_length(
          {std::make_pair(0, 0), std::make_pair(0, 1), std::make_pair(0, 2)}),
      -2);
  ASSERT_EQ(player_graph.get()->get_path_length(player_graph.get()->dijkstra(
                std::make_pair(0, 0), std::make_pair(0, 2))),
            -2);
}


/*
* Should connect the graph on connecting cell assignment.
* assigning the cell c to the player 0 (first player) should connect the player
graph
* Note that the grid is rotated (90 degree counter clock-wise) here for ascii
 drawing convenience.
                          _____
                         /     \
                   _____/       \
                  /     \       /
            _____/   0   \_____/
           /     \       /     \
     _____/       \_____/       \
    /     \       /     \       /
   /       \_____/   0   \_____/
   \       /     \       /     \
    \_____/   c   \_____/       \
    /     \       /     \       /
   /   0   \_____/       \_____/
   \       /     \       /     \
    \_____/       \_____/       \
    /     \       /     \       /
   /   0   \_____/       \_____/
   \       /     \       /
    \_____/       \_____/
    /     \       /
   /   0   \_____/
   \       /
    \_____/

 */

TEST(HexBoardTest, should_connect_the_graph_on_connecting_cell_assignment) {
  auto hex_board = mylib::HexBoard<int>(4, 4, 1); // builds one player config.
  // adding the first sub-graph to the player 0
  ASSERT_TRUE(hex_board.assign_cell_to_player(std::make_pair(0, 0), 0));
  ASSERT_TRUE(hex_board.assign_cell_to_player(std::make_pair(0, 1), 0));
  ASSERT_TRUE(hex_board.assign_cell_to_player(std::make_pair(0, 2), 0));
  // adding the a second sub-graph to the player 0
  ASSERT_TRUE(hex_board.assign_cell_to_player(std::make_pair(2, 2), 0));
  ASSERT_TRUE(hex_board.assign_cell_to_player(std::make_pair(2, 3), 0));

  auto player_graph = hex_board.get_player_graph(0);

  // assert that there is no path between cells (0,0) and (2, 3)
  ASSERT_EQ(player_graph.get()
                ->dijkstra(std::make_pair(0, 0), std::make_pair(2, 3))
                .size(),
            0);

  // adding the cell c (1,2)
  ASSERT_TRUE(hex_board.assign_cell_to_player(std::make_pair(1, 2), 0));
  auto dijkstra_path =
      player_graph.get()->dijkstra(std::make_pair(0, 0), std::make_pair(2, 3));
  ASSERT_EQ(dijkstra_path.size()-1, 5); // edges count
}