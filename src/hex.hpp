/* @modify date 2019-12-01 17:04:21
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

namespace mylib {

template <class T> class HexCell {
public:
  HexCell(std::size_t i_pos, std::size_t j_pos, T cell_content)
      : i_pos(i_pos), j_pos(j_pos), content(cell_content) {}
  std::pair<std::size_t, std::size_t> get_position() {
    return std::make_pair(i_pos, j_pos);
  }
  bool operator==(const HexCell<T> &other) {
    return get_position() == other.get_position();
  }

protected:
  T content;
  std::size_t i_pos, j_pos;
};

enum class HEX_DIRECTION { EAST, WEST, NORTH, SOUTH, NORTH_EAST, SOUTH_WEST };
enum class GAME_EDGE { EAST, WEST, NORTH, SOUTH };

// TODO: implement CELL (as abstract class) /EDGE_CELL/ GRID_CELL

class BaseCell {
  public:
   virtual bool operator == (const BaseCell &bc) =0;

};

/*
* A special cell type where every cell edge Hex cell on the board should be attached to
* we need one for each of the edges namely: WEST, NORTH, EAST, SOUTH they are special
* Cells/nodes to test if a player has connected the north to south, or the west to
*/
class FunnelCell: public BaseCell {

public:
  FunnelCell() {}
   virtual bool operator == (const BaseCell& other) {return false;}

};

template <class T>
class Cell: public BaseCell {
  public:
    Cell(std::pair<std::size_t, std::size_t> coordinates) : coordinates(coordinates){}
    Cell(size_t first, size_t second) { coordinates = std::make_pair(first, second);}

    std::pair<std::size_t, std::size_t> get_coordinate() const {return coordinates;}

    size_t first() const {return this->coordinates.first;}
    size_t second() const {return this->coordinates.second;}
    size_t row() { return this->first();}
    size_t col() { return this->second();}

    virtual bool operator == (const Cell& other) {
      return ((this->first() == other.first()) && (this->second() == other.second()));
    }
    virtual bool operator == (const BaseCell& other) {return false;}
  private:
    T value;
    std::pair<size_t, size_t> coordinates;

};


/*
* A Class encapsulating player data hex cell graph
*/
class Player {
  public:
    Player() {
       new Graph<std::shared_ptr<BaseCell>>;
    }
  private:
    std::unique_ptr<Graph<std::shared_ptr<BaseCell>>> graph;
};


template <class T>
class HexBoard {

  // A Hex board is represented by a grid of cell
  // as well as a vector of graphs for generality (might support multi-players >
  // 2 players) usually would contain 2 graphs (one for each player)

  std::size_t x_size, y_size;
  std::vector<std::vector<std::shared_ptr<Cell<T>>>>
      board; // should be a board of unique pointers to ensure that every cell
             // is assigned to a unique graph

  std::vector<std::unique_ptr<Graph<std::shared_ptr<BaseCell>>>>
      players_graphs; // a graph for each player

  std::unordered_map<std::pair<std::size_t, std::size_t>, std::size_t, boost::hash<std::pair<std::size_t, std::size_t>>>
      cell_to_player; // a map assigning each board cell to the player who has
                      // collected/marked it
  // Player graphs could be dynamically built from this map (on demand)

public:
  HexBoard(std::size_t x_size, std::size_t y_size, std::size_t players_nbr = 2)
      : x_size(x_size), y_size(y_size){
    board = std::vector<std::vector<std::shared_ptr<Cell<T>>>>(x_size, std::vector<std::shared_ptr<Cell<T>>>(y_size, nullptr));
    for (auto i=0; i<x_size; i++) {
      for (auto j=0; j<y_size; j++) {
        board[i][j] = std::make_shared<Cell<T>>( Cell<T>(i, j));
      }
    }
    // Initializa player graphs
    players_graphs.reserve(players_nbr);
    for (int i = 0; i < players_nbr; i++) {
      players_graphs.emplace_back(
          new Graph<std::shared_ptr<BaseCell>>);
    }
  }

  std::size_t players_nbr() {
    return players_graphs.size();
  }


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
      neighbor = within_the_board_pos(std::make_pair(cell.first(), cell.second() + 1));
      break;
    case (HEX_DIRECTION::WEST):
      neighbor = within_the_board_pos(std::make_pair(cell.first(), cell.second() - 1));
      break;
    case (HEX_DIRECTION::NORTH):
      neighbor = within_the_board_pos(std::make_pair(cell.first() - 1, cell.second()));
      break;
    case (HEX_DIRECTION::SOUTH):
      neighbor = within_the_board_pos(std::make_pair(cell.first() + 1, cell.second()));
      break;
    case (HEX_DIRECTION::NORTH_EAST):
      neighbor =
          within_the_board_pos(std::make_pair(cell.first() - 1, cell.second() + 1));
      break;
    case (HEX_DIRECTION::SOUTH_WEST):
      neighbor =
          within_the_board_pos(std::make_pair(cell.first() + 1, cell.second() - 1));
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
   * lookup for a cell int the cell to player map and return player index (if it exists)
   * @param cell: a cell location in the grid (cell) as std::pair<std::size_t,
   * std::size_t>
   * @return: an optional player index as std::pair<std::size_t, std::size_t>
   */
  boost::optional<std::size_t>
  get_player(std::pair<std::size_t, std::size_t> cell) {
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

  boost::optional<std::unique_ptr<Graph<std::shared_ptr<mylib::BaseCell>>> &>
  get_player_graph(std::size_t player_index) {
    if (player_index > players_graphs.size())
      return {};
    return players_graphs[player_index];
  }


  bool add_east_cell_to_player(std::shared_ptr<BaseCell> cell, std::size_t player_index){
    if (player_index < players_nbr()) {
      for (auto row: board) {
        auto edge_cell = row.front();
        // Connect the cell gived in param to last cell on each row (equivalent to connecting it to the east edge)
        auto edge = mylib::Edge<std::shared_ptr<BaseCell>>(cell, edge_cell, 0);
        players_graphs[player_index]->add_symmetric_edge(edge);
      }
     }
  }

  bool add_west_cell_to_player(std::shared_ptr<BaseCell> cell, std::size_t player_index){
    if (player_index < players_nbr()) {
      for (auto row: board) {
        auto edge_cell = row.back();
        // Connect the cell gived in param to last cell on each row (equivalent to connecting it to the west edge)
        auto edge = mylib::Edge<std::shared_ptr<BaseCell>>(edge_cell, cell, 0);
        players_graphs[player_index]->add_symmetric_edge(edge);
      }
     }
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
      // check if the cell is assined to another player
      if (cell_to_player.find(cell_pos) == cell_to_player.end()) {
        cell_to_player[cell_pos] = player_index;
        // maintain/build the edges of the player graph after adding the cell
        for (auto neighbor : all_neighbors(cell_pos)) {
          if ((get_player(neighbor) == player_index)) {
            // has to be a symmetric edge
            players_graphs[player_index]->add_symmetric_edge(
                mylib::Edge<std::shared_ptr<BaseCell>>(within_the_board(neighbor).get(), within_the_board(cell_pos).get(), 1));
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
