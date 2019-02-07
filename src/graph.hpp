#ifndef MYLIB_GRAPH
#define MYLIB_GRAPH
#include "edge.hpp"
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <ostream>
#include <random>
#include <set>
#include <stdexcept>
#include <vector>

namespace mylib {

template <class T> class Graph {
  T **graph;
  int size; // number of nodes
public:
  static inline const char *INFO = "mylib::Graph";
  int get_size() const { return size; }
  T get_item(int i, int j) const { return graph[i][j]; }
  void set_item(int i, int j, T item) { graph[i][j] = item; }
  void set_item_symmetric(int i, int j, T item) {
    graph[i][j] = item;
    graph[j][i] = item;
  }

  explicit Graph(Graph<T> const &g_source)
      : size(g_source.get_size()), graph(new T *[g_source.get_size()]) {
    // deep copy constructor
    for (int i = 0; i < size; ++i) {
      graph[i] = new T[size];
      for (int j = 0; j < size; j++) {
        graph[i][j] = g_source.get_item(i, j);
      }
    }
  }

  explicit Graph(Graph<T> *const g_source)
      : size(g_source->get_size()), graph(new T *[g_source->get_size()]) {
    // deep copy constructor
    for (int i = 0; i < this->size; ++i) {
      this->graph[i] = new T[this->size];
      for (int j = 0; j < size; j++) {
        this->graph[i][j] = g_source->get_item(i, j);
      }
    }
  }

  explicit Graph(int graph_size)
      : size(graph_size), graph(new T *[graph_size]) {
    for (int i = 0; i < size; ++i) {
      graph[i] = new T[size];
      for (int j = 0; j < i; ++j) {
        graph[i][j] = graph[j][i] = T();
      }
    }
  }

  Graph *filter(std::function<bool(T)> filter) {
    Graph *filtered_graph = new Graph(this->get_size());
    for (int i = 0; i < size; ++i) {
      // only under symmetric dist.
      for (int j = 0; j <= i; j++) {
        if (filter(get_item(i, j))) {
          filtered_graph->set_item(i, j, graph[i][j]);
          filtered_graph->set_item(j, i, graph[i][j]);
        }
      }
    }
    return filtered_graph;
  }

  void random_fill(std::function<T()> generator) {

    for (int i = 0; i < size; ++i) {
      for (int j = 0; j < i; j++) {
        graph[i][j] = graph[j][i] = generator();
      }
    }
  }

  /**
   * returns for directly reachable nodes (with defined direct age)
   */
  std::map<int, Edge> get_next_reachable(int source) {
    std::map<int, Edge> reachable;
    if (source < this->size) {
      for (int j = 0; j < this->size; j++) {
        if (this->graph[source][j].is_enabled()) {
          reachable.insert(std::make_pair(j, this->graph[source][j]));
        }
      }
    }
    return reachable;
  }

  /**
   * return the path length from a list of nodes (also check for cycles)
   */
  double get_path_length(std::vector<int> path) {
    double length = 0;
    if (path.size() > 1) {
      auto from = 0;
      std::set<int> nodes; // nodes to check cycles
      while (from < path.size() - 1) {
        length += graph[path[from]][path[from + 1]].get_distance();
        from += 1;
      }
    }
    return length;
  }

  /**
   * get the index of tje shortest path given a vector of paths
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
    for (int i = 0; i < g.get_size(); ++i) {
      out << "  ";
      for (int j = 0; j < g.get_size(); ++j) {
        out << g.get_item(i, j) << " ";
      }
      out << std::endl;
    }
    out << "--" << std::endl;
    return out;
  }

  std::vector<int> dijkstra(int starting_node_index, int final_node_index) {
    std::set<int> explored({starting_node_index}); // list of explored nodes
    std::vector<std::vector<int>> paths(
        {{starting_node_index}}); // paths to explore (extend)
   
    while (paths.size() > 0) {
      auto shortest_path_index = get_shortest_path(paths);
      auto shortest_path = paths[shortest_path_index];
      paths.erase(paths.begin() + shortest_path_index); // pop the shortest path to explore
      if (shortest_path.back() == final_node_index) {
        // found shortest path to destination (final node_index)
        return shortest_path;
      }
      // get the reachable nodes starting from last node of the path
      for (auto node : get_next_reachable(shortest_path.back())) {
        if (explored.find(node.first) == explored.end()) {
          // extend the path with the node
          auto path_to_extend = shortest_path;
          path_to_extend.push_back(node.first);
          paths.push_back(path_to_extend);
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
