/**
 * @author Wajih Ouertani
 * @email wajih.ouertani@gmail.com
 */

#ifndef MYLIB_GRAPH
#define MYLIB_GRAPH

#include <functional>
#include <iostream>
#include <unordered_map>
#include <ostream>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>
#include <queue>

#include <boost/optional.hpp>
#include "utils.hpp"
#include "edge.hpp"

namespace mylib {

template <class Node> class Graph {
protected:
  std::unordered_map<Node, std::unordered_map<Node, Edge<Node>>> graph;
  std::set<Node> to_nodes; // storing edges end (to(s)) nodes (as from could be
                           // collected from graph)

public:
  static constexpr const char *INFO = "mylib::Graph";


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
        add_edge(new Edge<Node>(end_1, end_2));
      }
    }
  }

  /** a Constructor using Node vector a generator of edges as a function of (Node from_, Node to_)
   */
  explicit Graph(std::vector<Node> nodes,
                   std::function<Edge<Node>(Node, Node)> generator, double density=1) {
    for (auto from_ : nodes) {
      for (auto to_ : nodes) {
        // generate edge accordingly with density parameter
        if (utils::random<double>(0, 1) <= density) {
          add_edge(generator(from_, to_));
        }
      }
    }
  }

  /**
  return size or more precisely the number of edges
  */
  unsigned int get_edges_number() const {
    unsigned int size=0;
    for (auto it=graph.begin(); it != graph.end(); it++) {
      size += it->second.size();
    }
    return size;
  }
  unsigned int E() const {
    return this->get_edges_number();
  }

  /** return vertices/nodes number
   */

  unsigned int get_nodes_number() const {
    std::set<Node> nodes;
    for (auto it=graph.begin(); it != graph.end(); it++) {
      nodes.insert(it->first);
      for (auto node:it->second) {
        nodes.insert(node.first);
      }
    }
    return nodes.size();
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
  boost::optional<Edge<Node>> get_edge(Node from_, Node to_) const {
    auto source_edges = graph.find(from_);
    if (source_edges != graph.end()) {
      auto source_dest_edge = source_edges->second.find(to_);
      if (source_dest_edge != source_edges->second.end()) {
        return {source_dest_edge->second};
      }
    }
    return {};
  }

  /**
   * return edge value if the edge exists
   */

  boost::optional<double> get_edge_value(Node from_, Node to_) {
    auto edge = get_edge(from_, to_);
    if (edge) {
      return edge.get().get_distance(); // TODO refactor get_distance to get_value (more generic)
    }
    return {};
  }

  /**
   * set edge value (if edge exists/already created)
   * does not create the edge if  does not exist
  */
  boost::optional<bool> set_edge_value(Node from_, Node to_, double value) {

  auto source_edges = graph.find(from_);
    if (source_edges != graph.end()) {
      auto source_dest_edge = source_edges->second.find(to_);
      if (source_dest_edge != source_edges->second.end()) {
        return {source_dest_edge->second.set_value(value)};
      }
    }
    return {};
  }


  void add_edge(Edge<Node> edge) {
    // check the first end
    auto first_edge_end =
        graph.find(edge.from()); // search the first end in the graph
    if (first_edge_end == graph.end()) {
      // the first edge end is not found (not connected to any of the nodes in
      // the graph)
      std::unordered_map<Node, Edge<Node>>
          first_end_init; // initializa a connection map
      first_end_init.insert(std::make_pair(edge.to(), edge));
      graph[edge.from()] = first_end_init;
    } else {
      // the first edge is already in the graph so update (overwrite) with
      // second end
      first_edge_end->second.insert(std::make_pair(edge.to(), edge));
    }
    // update nodes
    this->to_nodes.insert(edge.to());
  }

  void add_symmetric_edge(Edge<Node> edge) {
    add_edge(edge);
    add_edge(edge.sym_reverse());
  }

  /**
   * filter the graph using a filtering function (functional style)
   */
  Graph *filter(std::function<bool(Edge<Node>)> filter) {
    Graph *filtered_graph = new Graph();
    for (auto from_ : this->graph) {
      for (auto to_ : from_.second) {
        if (filter(to_.second)) {
          filtered_graph->add_edge(to_.second);
        }
      }
    }
    return filtered_graph;
  }

  /**
   * return directly reachable nodes (with defined direct edge)
   */
  boost::optional<std::unordered_map<Node, Edge<Node>>>
  neighbors(Node source) {
    auto connected_nodes = graph.find(source);
    if (connected_nodes != graph.end()) {
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
          length += edge_.get().get_distance();
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

  /**
   *  Dijkstra shortest path search implementation
   *  returns path from  starting_node to final_node
   * */

  std::vector<Node> dijkstra(Node starting_node_index, Node final_node_index) {
    std::set<Node> explored(
        {starting_node_index}); // list of explored nodes (closed set)

    auto compare = [] (std::pair<std::vector<Node>, double> left, std::pair<std::vector<Node>, double> right) {
      return left.second > right.second;
    };

    std::priority_queue<std::pair<std::vector<Node>, double>, std::vector<std::pair<std::vector<Node>, double>>,
                      decltype(compare)> paths(compare);
    paths.push(std::make_pair<std::vector<Node>, double>({starting_node_index}, 0.)); // paths to explore (extend)

    while (paths.size() > 0) {
      auto shortest_path = paths.top();
      paths.pop();

      if (shortest_path.first.back() == final_node_index) {
        // found shortest path to destination (final node_index)
        return shortest_path.first;
      }
      // get the reachable/neighbor nodes starting from last node of the path
      auto next_reachable = neighbors(shortest_path.first.back());
      if (next_reachable) {
        for (auto node : next_reachable.value()) {
          if (explored.find(node.first) == explored.end()) {
            // extend the path with the node (as the node is not yet in the
            // explored set)
            auto path_to_extend = shortest_path.first;
            path_to_extend.push_back(node.first);
            // update the paths priority queue
            paths.push(std::make_pair(path_to_extend, get_path_length(path_to_extend)));
          }
        }
      }
      // update the explored set
      explored.insert(shortest_path.first.back());
    }
    return {};
  }
};
} // namespace mylib
#endif
