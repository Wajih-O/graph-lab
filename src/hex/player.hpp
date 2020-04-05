/**
 * @author Wajih Ouertani
 * @email wajih.ouertani@gmail.com
 * @create date 2020-04-05 02:49:39
 * @modify date 2020-04-05 04:37:16
 */

#ifndef MYLIB_HEX_PLAYER
#define MYLIB_HEX_PLAYER

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

namespace mylib {

/**
 * A Class encapsulating player data hex cell graph
 */

class Player {
public:
  Player() {
    graph = std::unique_ptr<Graph<std::shared_ptr<BaseCell>>>(new Graph<std::shared_ptr<BaseCell>>());
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

bool is_west_east_connected() {
  bool connected = graph->dijkstra(get_west_funnel_cell(), get_east_funnel_cell()).size() > 0;
  if (connected) {
    // TODO: replace with proper logging
    std::cout << "west -> east connected !!!" << std::endl;
  }
  return connected;
}

bool is_north_south_connected() {
  bool connected = graph->dijkstra(get_north_funnel_cell(), get_south_funnel_cell()).size() > 0;
  if (connected) {
    // TODO: replace with proper logging
    std::cout << "north -> south connected !!!" << std::endl;
  }
  return connected;
}

bool is_board_edges_connected() {
  return (is_north_south_connected() || is_west_east_connected());
}
// TODO: get the longest path (a player should try to extend in the next move) as a strategy

private:

  std::unique_ptr<Graph<std::shared_ptr<BaseCell>>> graph;
  std::shared_ptr<FunnelCell> south_funnel_cell;
  std::shared_ptr<FunnelCell> north_funnel_cell;
  std::shared_ptr<FunnelCell> east_funnel_cell;
  std::shared_ptr<FunnelCell> west_funnel_cell;
};
}

#endif