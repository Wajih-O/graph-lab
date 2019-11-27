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

TEST(HexBoardTest, should_build_graph_properly_for_the_first_player) {
  auto hex_board = mylib::HexBoard<int>(4, 4, 1); // builds one player config.
  ASSERT_TRUE(hex_board.assign_cell_to_player(std::make_pair(0, 0), 0));
  ASSERT_TRUE(hex_board.assign_cell_to_player(std::make_pair(0, 1), 0));
  ASSERT_TRUE(hex_board.assign_cell_to_player(std::make_pair(0, 2), 0));

  auto player_graph =  hex_board.get_player_graph(0);
  ASSERT_EQ(player_graph.get().get_edges_number(), 2);

  ASSERT_EQ(player_graph.get().get_path_length({std::make_pair(0, 0), std::make_pair(0, 1), std::make_pair(0, 2)}), 2);
  ASSERT_EQ(player_graph.get().dijkstra(std::make_pair(0, 0), std::make_pair(0, 2)).size(), 3);
}