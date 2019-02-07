#include "graph.hpp"
#include "list.hpp"
#include <iostream>

void list_lab() {
  mylib::List<int> *l = new mylib::List<int>(2);
  std::cout << *l->prepend(3)->prepend(4)->prepend(5) << std::endl;

  mylib::List<double> *double_list = new mylib::List<double>(2);
  std::cout << *double_list->prepend(7)->prepend(8.4)->prepend(5) << std::endl;
  std::cout << *l->get_tail() << std::endl;
  std::cout << *l << std::endl;
}

int graph_lab() {
  mylib::Graph<mylib::Edge> *graph = new mylib::Graph<mylib::Edge>(10);
  graph->random_fill(mylib::Edge::generate(3, 6));
  std::cout << *graph << std::endl;
  std::cout << *graph->filter(mylib::Edge::more_than_filter(5.)) << std::endl;
  std::cout << *graph->filter(mylib::Edge::less_than_filter(5.)) << std::endl;
}

int graph_lab_dijkstra_wikipedia() {
  mylib::Graph<mylib::Edge> *graph = new mylib::Graph<mylib::Edge>(6);
  // Fill the graph
  // https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm

  graph->set_item_symmetric(1, 0, mylib::Edge(true, 50.));
  graph->set_item_symmetric(2, 1, mylib::Edge(true, 17.));
  graph->set_item_symmetric(5, 1, mylib::Edge(true, 3.));
  graph->set_item_symmetric(2, 0, mylib::Edge(true, 9.));
  graph->set_item_symmetric(3, 2, mylib::Edge(true, 11.));
  graph->set_item_symmetric(4, 3, mylib::Edge(true, 6.));
  graph->set_item_symmetric(5, 0, mylib::Edge(true, 14.));
  graph->set_item_symmetric(5, 2, mylib::Edge(true, 2.));
  graph->set_item_symmetric(5, 4, mylib::Edge(true, 9.));

  // graph->random_fill(mylib::Edge::generate(3, 6));

  std::cout << *graph << std::endl;
  mylib::Graph<mylib::Edge> * copy_graph = new mylib::Graph<mylib::Edge>(graph);
  std::cout << *copy_graph->filter(mylib::Edge::less_than_filter(10));

  for (auto item:graph->get_next_reachable(3)) {
    std::cout << item.first << " edge ==" << item.second << std::endl;
  }
  // check path length 
  std::cout << graph->get_path_length({2,3,4}) << std::endl;

  for (auto item: graph->dijkstra(1, 0)) {
    std::cout << item << std::endl;
  }
}

int main() {
  // graph_lab();
  graph_lab_dijkstra_wikipedia();
}