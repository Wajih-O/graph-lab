#include "edge.hpp"
#include "graph.hpp"
#include "list.hpp"
#include <iostream>
#include <string>

void list_lab() {
  mylib::List<int> *l = new mylib::List<int>(2);
  std::cout << *l->prepend(3)->prepend(4)->prepend(5) << std::endl;

  mylib::List<double> *double_list = new mylib::List<double>(2);
  std::cout << *double_list->prepend(7)->prepend(8.4)->prepend(5) << std::endl;
  std::cout << *l->get_tail() << std::endl;
  std::cout << *l << std::endl;
}

int graph_lab() {
  std::vector<int> nodes = {1, 2, 3, 4, 5};
  mylib::Graph<int> *graph =
      new mylib::Graph<int>(nodes, mylib::Edge<int>::generate(1., 10.));
  std::cout << *graph << std::endl;
  std::cout << *graph->filter(mylib::Edge<int>::more_than_filter(5.))
            << std::endl;
  std::cout << *graph->filter(mylib::Edge<int>::less_than_filter(5.))
            << std::endl;
}

void graph_example() {
  mylib::Graph<int> *graph = new mylib::Graph<int>();

  // Fill the graph https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm

  graph->add_symmetric_edge(mylib::Edge<int>(1, 2, true, 7.));
  graph->add_symmetric_edge(mylib::Edge<int>(1, 3, true, 9.));
  graph->add_symmetric_edge(mylib::Edge<int>(1, 6, true, 14.));

  graph->add_symmetric_edge(mylib::Edge<int>(2, 1, true, 7.));
  graph->add_symmetric_edge(mylib::Edge<int>(2, 3, true, 10.));
  graph->add_symmetric_edge(mylib::Edge<int>(2, 4, true, 15.));

  graph->add_symmetric_edge(mylib::Edge<int>(3, 6, true, 2.));
  graph->add_symmetric_edge(mylib::Edge<int>(3, 4, true, 6.));

  graph->add_symmetric_edge(mylib::Edge<int>(4, 5, true, 6.));
  graph->add_symmetric_edge(mylib::Edge<int>(5, 6, true, 9.));

  // std::cout << *graph << std::endl;
  // std::cout << "filter less than 10:" << std::endl
  //           << *graph->filter(mylib::Edge<int>::less_than_filter(10));

  // auto reachable_from_node_3 = graph->get_next_reachable(3);
  // if (reachable_from_node_3) {
  //   for (auto item : reachable_from_node_3.value()) {
  //     std::cout << item.first << " edge ==" << item.second << std::endl;
  //   }
  // }

  // check path length
  // std::cout << graph->get_path_length({1, 2, 4, 5}) << std::endl;
  auto shortest_path = graph->dijkstra(1, 5);
  for (auto item = shortest_path.begin(); item < shortest_path.end() -1; item++ ) {
    std::cout << *item << "-->" ;
  }
  std::cout << *(shortest_path.end()-1) << std::endl;

}

/**
 *  Example with Lina
 */
 void lina_graph() {
   mylib::Graph<std::string> graph;
   graph.add_symmetric_edge(mylib::Edge<std::string>("A", "B", true, 2.0));
   graph.add_symmetric_edge(mylib::Edge<std::string>("B", "C", true, 3.0));
   graph.add_symmetric_edge(mylib::Edge<std::string>("B", "D", true, 4.0));

     std::cout << &graph << std::endl;
 }

/**
 *  string graph example
 */
void string_graph() {

  mylib::Graph<std::string> *graph = new mylib::Graph<std::string>();
  graph->add_symmetric_edge(
      mylib::Edge<std::string>("node_0", "node_3", true, 50.));
  graph->add_symmetric_edge(
      mylib::Edge<std::string>("node_3", "n_5", true, 17.));
  graph->add_symmetric_edge(mylib::Edge<std::string>("n_5", "n_2", true, 3.));
  graph->add_symmetric_edge(
      mylib::Edge<std::string>("node_3", "node_0", true, 9.));

  // check path length
  std::cout << graph->get_path_length({"node_0", "node_3", "n_5", "n_2"})
            << std::endl;

  auto reachable_from_node_3 = graph->get_next_reachable("node_3");
  std::cout << *graph << std::endl;
}

int main() {
  // graph_lab();
  graph_example();
  string_graph();
}