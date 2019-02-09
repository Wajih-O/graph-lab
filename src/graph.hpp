#ifndef MYLIB_GRAPH
#define MYLIB_GRAPH

#include "edge.hpp"
#include <functional>
#include <iostream>
#include <map>
#include <optional>
#include <ostream>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

namespace mylib {

template <class Node> class Graph {
protected:
  std::unordered_map<Node, std::unordered_map<Node, Edge<Node>>> graph;
  std::set<Node> to_nodes; // storing edges end (to(s)) nodes (as from could be
                           // collected from graph)

public:
  static inline const char *INFO = "mylib::Graph";
  /**
  return size or more precisely the number of edges
  */
  unsigned int get_size() const {
    unsigned int size;
    for (auto it = graph.begin(); it < graph.end(); it++) {
      size += it->second.size();
    }
    return size;
  }

  /*
   * return the graph
   **/
  std::unordered_map<Node, std::unordered_map<Node, Edge<Node>>> get_graph() {
    return graph;
  }

  /**
   * get edge given source and dest nodes
   */
  std::optional<Edge<Node>> get_edge(Node from_, Node to_) const {
    auto source_edges = graph.find(from_);
    if (source_edges != graph.end()) {
      auto source_dest_edge = source_edges->second.find(to_);
      if (source_dest_edge != source_edges->second.end()) {
        return {source_dest_edge->second};
      }
    }
    return {};
  }

  void set_item(Edge<Node> edge) {
    // check the first end
    auto first_edge_end =
        graph.find(edge.get_end_1()); // search the first end in the graph
    if (first_edge_end == graph.end()) {
      // the first edge end is not found (not connected to any of the nodes in
      // the graph)
      std::unordered_map<Node, Edge<Node>>
          first_end_init; // initializa a connection map
      first_end_init.insert(std::make_pair(edge.get_end_2(), edge));
      graph[edge.get_end_1()] = first_end_init;
    } else {
      // the first edge is already in the graph so update (overwrite) with
      // second end
      first_edge_end->second.insert(std::make_pair(edge.get_end_2(), edge));
    }
    // update nodes
    this->to_nodes.insert(edge.get_end_2());
  }

  void set_item_symmetric(Edge<Node> edge) {
    set_item(edge);
    set_item(edge.sym_reverse());
  }

  /**
   * Constructors
   */
  Graph() {}
  explicit Graph(Graph<Node> const &g_source) { graph = g_source.graph; }
  explicit Graph(Graph<Node> *const g_source) { graph = g_source->graph; }
  /**
   * Construct a collapsed (d(x,y)==0 for all x, y in graph nodes) complete
   * graph from a list of nodes
   */
  explicit Graph(std::vector<Node> nodes) {
    for (auto end_1 : nodes) {
      for (auto end_2 : nodes) {
        set_item(new Edge<Node>(end_1, end_2));
      }
    }
  }

  explicit Graph(std::vector<Node> nodes,
                   std::function<Edge<Node>(Node, Node)> generator) {
    for (auto from_ : nodes) {
      for (auto to_ : nodes) {
        set_item(generator(from_, to_));
      }
    }
  }

  /**
   * filter the graph using a filtering function (functional style)
   */
  Graph *filter(std::function<bool(Edge<Node>)> filter) {
    Graph *filtered_graph = new Graph();
    for (auto from_ : this->graph) {
      for (auto to_ : from_.second) {
        if (filter(to_.second)) {
          filtered_graph->set_item(to_.second);
        }
      }
    }
    return filtered_graph;
  }

  /**
   * return directly reachable nodes (with defined direct edge)
   */
  std::optional<std::unordered_map<Node, Edge<Node>>>
  get_next_reachable(Node source) {

    auto connected_nodes = graph.find(source);
    if (connected_nodes != graph.end()) {
      // debug print
      std::cout << "source: " << source;
      for (auto node : connected_nodes->second) {
        std::cout << " --> " << node.first << " (" << node.second << ")";
      }
      std::cout << std::endl;
      return connected_nodes->second;
    }
    return {};
  }

  /**
   * return the path length from a list of nodes (also check for cycles)
   */
  double get_path_length(std::vector<Node> path) {
    double length = 0;
    if (path.size() > 1) {
      auto from = 0;
      // std::set<int> nodes; // nodes to check cycles
      while (from < path.size() - 1) {
        auto edge_ = get_edge(path[from], path[from + 1]);
        if (edge_) {
          length += edge_.value().get_distance();
        } else {
          return -1; // as there is no path (disrupted/cut)
        }
        from += 1;
      }
    }
    return length;
  }

  /**
   * get the index of the shortest path given a vector of paths
   */
  int get_shortest_path(std::vector<std::vector<int>> paths) {
    if (paths.size() > 0) {
      auto shortest = get_path_length(paths[0]); // initialize shortest path
      int shortest_index = 0;
      for (unsigned int index = 1; index < paths.size(); index++) {
        if (get_path_length(paths[index]) < shortest) {
          shortest_index = index;
          shortest = get_path_length(paths[index]);
        }
      }
      return shortest_index;
    }
    throw std::runtime_error("paths list/vector is emtpty");
  }

  friend std::ostream &operator<<(std::ostream &out, const Graph &g) {
    out << Graph::INFO << std::endl;
    out << "--";
    out << "  Dist. matrix " << std::endl;
    for (auto from_ : g.graph) {
      out << from_.first;
      for (auto to_ : from_.second) {
        out << " -> " << to_.first << "("<<to_.second.get_distance() << ")";
      }
      out << std::endl;
    }
    out << "--" << std::endl;
    return out;
  }

  std::vector<Node> dijkstra(Node starting_node_index, Node final_node_index) {
    std::set<Node> explored(
        {starting_node_index}); // list of explored nodes (closed set)
    std::vector<std::vector<Node>> paths(
        {{starting_node_index}}); // paths to explore (extend)

    while (paths.size() > 0) {
      auto shortest_path_index = get_shortest_path(paths);
      auto shortest_path = paths[shortest_path_index];
      paths.erase(paths.begin() +
                  shortest_path_index); // pop the shortest path to explore
      if (shortest_path.back() == final_node_index) {
        // found shortest path to destination (final node_index)
        return shortest_path;
      }
      // get the reachable nodes starting from last node of the path
      auto next_reachable = get_next_reachable(shortest_path.back());
      if (next_reachable) {
        for (auto node : next_reachable.value()) {
          if (explored.find(node.first) == explored.end()) {
            // extend the path with the node (as the node is not yet in the
            // explored set)
            auto path_to_extend = shortest_path;
            path_to_extend.push_back(node.first);
            paths.push_back(path_to_extend);
          }
        }
      }
      // update the explored set
      explored.insert(shortest_path.back());
    }
    return {};
  }
};
} // namespace mylib
#endif
