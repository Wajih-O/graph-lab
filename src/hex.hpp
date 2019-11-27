/* @modify date 2019-11-27 04:14:02
 * @author Wajih Ouertani
 * @email wajih.ouertani@gmail.com
 */

#ifndef MYLIB_HEX_BOARD
#define MYLIB_HEX_BOARD

#include <algorithm>
#include <unordered_map>
#include <utility>
#include <vector>

#include <boost/functional/hash.hpp>
#include <boost/optional.hpp>


#include "graph.hpp"
#include "utils.hpp"

namespace mylib {

template <class  T> class HexCell {
public:
  HexCell(std::size_t i_pos, std::size_t j_pos, T cell_content)
      : i_pos(i_pos), j_pos(j_pos), content(cell_content) {}
  std::pair<std::size_t, std::size_t> get_position() { return std::make_pair(i_pos, j_pos); }
  bool operator==(const HexCell<T> &other) {
    return get_position() == other.get_position();
  }

protected:
  T content;
  std::size_t i_pos, j_pos;
};

template <class T> class HexBoard {

  // A Hex board is represented by a grid of cell
  // as well as a vector of graphs for generality (might support multi-players >
  // 2 players) usually would contain 2 graphs (one for each player)

  std::size_t x_size, y_size, players_nbr;
  std::vector<std::vector<T>>
      board; // should be a board of unique pointers to ensure that every cell
             // is assigned to a unique graph
  std::vector<Graph<std::pair<std::size_t, std::size_t>>> players_graphs; // a graph for each player
  std::unordered_map<std::pair<std::size_t, std::size_t>, std::size_t, boost::hash<std::pair<std::size_t,std::size_t>> > cell_to_player; // a map assigning each board cell to the player who has collected/marked it
  // a graph could be dynamically built from this structure (on demand)
public:
  HexBoard(std::size_t x_size, std::size_t y_size, std::size_t players_nbr=2 )
      : x_size(x_size), y_size(y_size), players_nbr(players_nbr) {
    board = std::vector<std::vector<T>>(x_size, std::vector<T>(y_size, T()));
    players_graphs = std::vector<Graph<std::pair<std::size_t, std::size_t>>>(players_nbr, Graph<std::pair<std::size_t, std::size_t>>());
  }


  bool within_the_board(std::pair<std::size_t, std::size_t> cell_location) {
    return (cell_location.first < x_size) && (cell_location.second < y_size);
  }

  /*
  * lookup for a cell int the cell to player map and return player if it exist
  */
  boost::optional<std::size_t> get_player(std::pair<std::size_t, std::size_t> cell) {
    auto iter = cell_to_player.find(cell);
    if (iter != cell_to_player.end()) {
      return iter->second;
    }
  return {};
  }

  boost::optional<Graph<std::pair<std::size_t, std::size_t>>> get_player_graph(std::size_t player_index) {
    if (player_index > players_graphs.size()) return {};
    return players_graphs[player_index];
  }

  bool assign_cell_to_player(std::pair<std::size_t,std::size_t> cell, std::size_t player_index) {
    if ((within_the_board(cell)) && (player_index<players_nbr)) {
      cell_to_player[cell] = player_index;
      // maintain/build the edges of the player graph after adding the cell
      auto east_neighbor = std::make_pair(cell.first, cell.second+1);
      auto west_neighbor = std::make_pair(cell.first, cell.second-1);

      if (within_the_board(west_neighbor) && (get_player(west_neighbor) == player_index)) {
        players_graphs[player_index].add_edge(mylib::Edge<std::pair<std::size_t,std::size_t>>(west_neighbor, cell, 1));
      }
    return true;
    }
  return false;
  }

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
