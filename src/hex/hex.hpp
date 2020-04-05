/* @modify date 2020-04-05 03:40:46
 * @author Wajih Ouertani
 * @email wajih.ouertani@gmail.com
 */

#ifndef MYLIB_HEX_BOARD
#define MYLIB_HEX_BOARD

#include <algorithm>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

#include <boost/functional/hash.hpp>
#include <boost/optional.hpp>

#include "graph.hpp"
#include "utils.hpp"

#include "hex/cell.hpp"
#include "hex/player.hpp"

namespace mylib {

template <class T> class HexBoard {

  // A Hex board is represented by a grid of cell
  // as well as a vector of graphs for generality (might support multi-players >
  // 2 players) usually would contain 2 graphs (one for each player)

  std::size_t x_size, y_size;
  std::vector<std::vector<std::shared_ptr<Cell<T>>>>
      board; // should be a board of unique pointers to ensure that every cell
             // is assigned to a unique graph

  std::unordered_map<std::pair<std::size_t, std::size_t>, std::size_t,
                     boost::hash<std::pair<std::size_t, std::size_t>>>
      cell_to_player; // a map assigning each board cell to the player who has
                      // collected/marked it
  // Player graphs could be dynamically built from this map (on demand)


 /*
   * Connect a player special edge/funnel cell to the board edges
   * @param player_index: player index
   * @return: player graph for the relative player index
   */

  bool connect_east_funnel(Player& player) {
      for (auto row : board) {
       player.connect_east(row.front());
      }
  }

 bool connect_west_funnel(Player& player) {
      for (auto row : board) {
       player.connect_west(row.back());
      }
  }

  bool connect_north_funnel(Player& player) {
      for (auto cell: board.front()) {
       player.connect_north(cell);
      }
  }

 bool connect_south_funnel(Player& player) {
      for (auto cell : board.back()) {
       player.connect_south(cell);
      }
  }

public:
 std::vector<Player> players;
  HexBoard(std::size_t x_size, std::size_t y_size, std::size_t players_nbr = 2)
      : x_size(x_size), y_size(y_size) {
    board = std::vector<std::vector<std::shared_ptr<Cell<T>>>>(
        x_size, std::vector<std::shared_ptr<Cell<T>>>(y_size, nullptr));
    for (auto i = 0; i < x_size; i++) {
      for (auto j = 0; j < y_size; j++) {
        board[i][j] = std::make_shared<Cell<T>>(Cell<T>(i, j));
      }
    }
    for (auto i = 0; i < players_nbr; i++) {
      players.push_back(Player());
      // Connect player funnel/special edge cells to the board
      connect_east_funnel(players.back());
      connect_west_funnel(players.back());
      connect_north_funnel(players.back());
      connect_south_funnel(players.back());
    }
  }

  std::size_t players_nbr() { return players.size(); }

  /*
  * Helper methods to get neighbors of a cell (C) in East (E), west (W), North,
  North-East (N-E), South (S), South West (SW)
  * Note that the grid is rotated (90 degree counter clock-wise) here for ascii
  drawing convenience,

            /     \       /     \
      _____/       \_____/       \_____
     /     \       /     \       /     \
    /       \_____/   E   \_____/       \
    \       /     \       /     \       /
     \_____/  N-E  \_____/   S   \_____/
     /     \       /     \       /     \
    /       \_____/   C   \_____/       \
    \       /     \       /     \       /
     \_____/   N   \_____/  S-W  \_____/
     /     \       /     \       /
    /       \_____/   W   \_____/
    \       /     \       /
     \_____/       \_____/
     /     \       /
    /       \_____/
    \       /
     \_____/

  * returns the next neighbor of a cell for a given direction, if the grid
  limits are not reached (the neighbor is within the grid)
  */

  boost::optional<std::pair<std::size_t, std::size_t>>
  grid_neighbor(Cell<T> cell, HEX_DIRECTION direction) {
    boost::optional<std::pair<std::size_t, std::size_t>> neighbor = {};
    switch (direction) {
    case (HEX_DIRECTION::EAST):
      neighbor =
          within_the_board_pos(std::make_pair(cell.first(), cell.second() + 1));
      break;
    case (HEX_DIRECTION::WEST):
      neighbor =
          within_the_board_pos(std::make_pair(cell.first(), cell.second() - 1));
      break;
    case (HEX_DIRECTION::NORTH):
      neighbor =
          within_the_board_pos(std::make_pair(cell.first() - 1, cell.second()));
      break;
    case (HEX_DIRECTION::SOUTH):
      neighbor =
          within_the_board_pos(std::make_pair(cell.first() + 1, cell.second()));
      break;
    case (HEX_DIRECTION::NORTH_EAST):
      neighbor = within_the_board_pos(
          std::make_pair(cell.first() - 1, cell.second() + 1));
      break;
    case (HEX_DIRECTION::SOUTH_WEST):
      neighbor = within_the_board_pos(
          std::make_pair(cell.first() + 1, cell.second() - 1));
      break;
    }
    return neighbor;
  }

  /*
   * Return a set of neighbor cell (position/coordinate)
   * @param cell: (cell of interest position/coordinate as
   * std::pair<std::size_t, std::size_t>) return: positions within the grid of
   * the neighbors cells of (cell) as  std::set<std::pair<std::size_t,
   * std::size_t>>
   */
  std::set<std::pair<std::size_t, std::size_t>>
  all_neighbors(std::pair<std::size_t, std::size_t> cell) {
    std::set<std::pair<std::size_t, std::size_t>>
        grid_neighbors; // next neighbors in the grid
    for (auto direction :
         {HEX_DIRECTION::EAST, HEX_DIRECTION::WEST, HEX_DIRECTION::NORTH,
          HEX_DIRECTION::SOUTH, HEX_DIRECTION::NORTH_EAST,
          HEX_DIRECTION::SOUTH_WEST}) {
      auto neighbor = grid_neighbor(cell, direction);
      if (neighbor) {
        grid_neighbors.insert(neighbor.get());
      }
    }
    return grid_neighbors;
  }

  bool is_within_the_board(std::pair<std::size_t, std::size_t> const &cell) {
    return (cell.first < x_size) && (cell.second < y_size);
  }

  bool is_within_the_board(std::shared_ptr<Cell<T>> cell) {
    return (cell.first() < x_size) && (cell.second() < y_size);
  }

  boost::optional<std::shared_ptr<Cell<T>>>
  within_the_board(std::pair<std::size_t, std::size_t> const &cell) {
    if (is_within_the_board(cell)) {
      return board[cell.first][cell.second];
    }
    return {};
  }

  boost::optional<std::pair<std::size_t, std::size_t>>
  within_the_board_pos(std::pair<std::size_t, std::size_t> const &cell_pos) {
    if (is_within_the_board(cell_pos)) {
      return std::make_pair(cell_pos.first, cell_pos.second);
    }
    return {};
  }

  /*
   * lookup for a cell int the cell to player map and return player index (if it
   * exists)
   * @param cell: a cell location in the grid (cell) as std::pair<std::size_t,
   * std::size_t>
   * @return: an optional player index as std::pair<std::size_t, std::size_t>
   */
  boost::optional<std::size_t>
  get_cell_player(std::pair<std::size_t, std::size_t> cell) {
    auto iter = cell_to_player.find(cell);
    if (iter != cell_to_player.end()) {
      return iter->second;
    }
    return {};
  }

  /*
   * lookup for a (unique ptr to) player graph given a player
   * @param player_index: player index
   * @return: player graph for the relative player index
   */

  boost::optional<Player> get_player(std::size_t player_index) {
    if (player_index > players_nbr())
      return {};
    return players.at(player_index);
  }


boost::optional<std::unique_ptr<Graph<std::shared_ptr<BaseCell>>>&> get_player_graph(std::size_t player_index) {
    if (player_index > players_nbr())
      return {};
    return players.at(player_index).get_graph();
  }





  /*
   * Assign (allocate) the cell to a given player who played/selected the cell
   * @param cell: cell to assign to the player
   * @param player_index: player to whom the cell will be assigned (given)
   * @return bool: whether the move is valid and finalized (cell assigned to the
   * player) or not update the player (player_index) with the new edges a
   * consequence of adding the cell
   */

  bool assign_cell_to_player(std::pair<std::size_t, std::size_t> cell_pos,
                             std::size_t player_index) {
    // Check if the player index is valid, the cell is within the grid and not
    // already assigned to another player.
    if ((is_within_the_board(cell_pos)) && (player_index < players_nbr())) {
      // check if the cell is assigned to another player
      if (cell_to_player.find(cell_pos) == cell_to_player.end()) {
        cell_to_player[cell_pos] = player_index;
        // maintain/build the edges of the player graph after adding the cell
        for (auto neighbor : all_neighbors(cell_pos)) {
          if ((get_cell_player(neighbor) == player_index)) {
            // has to be a symmetric edge
            players[player_index].get_graph()->add_symmetric_edge(
                Edge<std::shared_ptr<BaseCell>>(
                    within_the_board(neighbor).get(),
                    within_the_board(cell_pos).get(), 1));
          }
        }
        return true;
      }
    }
    return false;
  }

  /*
   * A small sanity check of the grid params.
   */

  bool check() {
    auto check = (board.size() == x_size);
    for (auto row : board) {
      check &= (row.size() == y_size);
      if (!check) {
        return check;
      }
    }
    return check;
  }
};

} // namespace mylib
#endif
