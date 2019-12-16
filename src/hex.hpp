/* @modify date 2019-12-16 02:15:50
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
  virtual bool operator==(const BaseCell &bc) = 0;
};

/*
 * A special cell type where every cell edge Hex cell on the board should be
 * attached to we need one for each of the edges namely: WEST, NORTH, EAST,
 * SOUTH they are special Cells/nodes to test if a player has connected the
 * north to south, or the west to
 */
class FunnelCell : public BaseCell {

public:
  FunnelCell() {}
  virtual bool operator==(const BaseCell &other) { return false; }
   friend std::ostream &operator<<(std::ostream &out, const FunnelCell &cell) {
    out <<"(Funnel)"  << std::endl;
    return out ;
  }

};

template <class T> class Cell : public BaseCell {
public:
  Cell(std::pair<std::size_t, std::size_t> coordinates)
      : coordinates(coordinates) {}
  Cell(size_t first, size_t second) {
    coordinates = std::make_pair(first, second);
  }

  std::pair<std::size_t, std::size_t> get_coordinate() const {
    return coordinates;
  }

  size_t first() const { return this->coordinates.first; }
  size_t second() const { return this->coordinates.second; }
  size_t row() { return this->first(); }
  size_t col() { return this->second(); }

  virtual bool operator==(const Cell &other) {
    return ((this->first() == other.first()) &&
            (this->second() == other.second()));
  }
  virtual bool operator==(const BaseCell &other) { return false; }

  friend std::ostream &operator<<(std::ostream &out, const Cell<T> &cell) {
    out <<"(" << cell.first() << "," << cell.second() << ")"  << std::endl;
    return out;
  }

  friend std::ostream &operator<<(std::ostream &out, const std::shared_ptr<Cell<T>> &cell) {
    out <<"(" << cell->first() << "," << cell->second() << ")"  << std::endl;
    return out;
  }

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
    graph =std::unique_ptr<Graph<std::shared_ptr<BaseCell>>>(new Graph<std::shared_ptr<BaseCell>>());
    // Initialize special to be connected to the board
    south_funnel_cell = std::make_shared<FunnelCell>(FunnelCell());
    north_funnel_cell = std::make_shared<FunnelCell>(FunnelCell());
    east_funnel_cell = std::make_shared<FunnelCell>(FunnelCell());
    west_funnel_cell = std::make_shared<FunnelCell>(FunnelCell());
  }

  std::unique_ptr<Graph<std::shared_ptr<BaseCell>>>& get_graph() { return graph; }

   // Funnel cells accessors
   std::shared_ptr<FunnelCell> get_south_funnel_cell() const {return south_funnel_cell;}
   std::shared_ptr<FunnelCell> get_north_funnel_cell() const {return north_funnel_cell;}
   std::shared_ptr<FunnelCell> get_east_funnel_cell() const {return east_funnel_cell;}
   std::shared_ptr<FunnelCell> get_west_funnel_cell() const {return west_funnel_cell;}


  // the edge should not be symmetric here otherwise all the edges will be mutually connected with (0) cost
  void connect_west(std::shared_ptr<BaseCell> edge_cell) {
    graph->add_edge(
        Edge<std::shared_ptr<BaseCell>>( this->west_funnel_cell, edge_cell, 0));
  }

  void connect_east(std::shared_ptr<BaseCell> edge_cell) {
    graph->add_edge(
        Edge<std::shared_ptr<BaseCell>>(edge_cell, this->east_funnel_cell, 0));
  }

  void connect_north(std::shared_ptr<BaseCell> edge_cell) {
    graph->add_edge(
        Edge<std::shared_ptr<BaseCell>>(this->north_funnel_cell, edge_cell,  0));
  }

  void connect_south(std::shared_ptr<BaseCell> edge_cell) {
    graph->add_edge(
        Edge<std::shared_ptr<BaseCell>>(edge_cell, this->south_funnel_cell, 0));
  }


private:

  std::unique_ptr<Graph<std::shared_ptr<BaseCell>>> graph;
  std::shared_ptr<FunnelCell> south_funnel_cell;
  std::shared_ptr<FunnelCell> north_funnel_cell;
  std::shared_ptr<FunnelCell> east_funnel_cell;
  std::shared_ptr<FunnelCell> west_funnel_cell;
};

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
