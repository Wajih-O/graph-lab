#ifndef MYLIB_EDGE
#define MYLIB_EDGE

#include <functional>
#include <iomanip>
#include <ostream>
#include <random>

namespace mylib {
/**
 * src:
 * https://stackoverflow.com/questions/2704521/generate-random-double-numbers-in-c
 */
template <typename Numeric, typename Generator = std::mt19937>
Numeric random(Numeric from, Numeric to) {
  thread_local static Generator gen(std::random_device{}());

  using dist_type =
      typename std::conditional<std::is_integral<Numeric>::value,
                                std::uniform_int_distribution<Numeric>,
                                std::uniform_real_distribution<Numeric>>::type;

  thread_local static dist_type dist;

  return dist(gen, typename dist_type::param_type{from, to});
}

template <class Node> class Edge {
private:
  bool enabled;
  double distance;
  Node end_1, end_2; // two ends of the edge

public:
  /**
   * Get distance
   */
  double get_distance() const { return distance; }
  bool is_enabled() const { return enabled; }
  Node get_end_1() { return end_1; }
  Node get_end_2() { return end_2; }

  /**
   * Randomely generate edges
   * returns an edge for each call
   * where the distance is between lower and upper bounds
   */
  static std::function<Edge<Node>(Node, Node)>
  generate(double lower = 0., double upper = 1.) {
    return [&](Node from, Node to) {
      double dist = random<double>(lower, upper);
      return Edge<Node>(from, to, true, dist);
    };
  }

  /**
   * Generate (more than) filter given a threshold
   */
  static std::function<bool(Edge<Node>)> more_than_filter(double threshold) {
    return [&](Edge<Node> e) { return (e.get_distance() > threshold); };
  }

  /**
   * Generate (less than) filter given a threshold
   */
  static std::function<bool(Edge<Node>)> less_than_filter(double threshold) {
    return [&](Edge<Node> e) { return (e.get_distance() < threshold); };
  }

  /**
   * builds/return reverse (symmetric: same distance/undirected graph) Edge from
   * end_2 to end_1
   */
  Edge<Node> sym_reverse() {
    return  Edge<Node>(this->end_2, this->end_1, this->enabled,
                          this->distance);
  }

  Edge(Node end_1, Node end_2)
      : enabled(false), distance(0.0), end_1(end_1), end_2(end_2) {}

  Edge(Node end_1, Node end_2, bool _enabled, double _dist)
      : enabled(_enabled), distance(_dist), end_1(end_1), end_2(end_2) {}

  bool collapsed() { return distance == 0.0; }

  friend std::ostream &operator<<(std::ostream &out, const Edge &e) {
    if (e.is_enabled()) {
      out << e.get_distance(); //  << std::setw(3) << std::setprecision(3) <<
                               //  e.get_distance();
    } else {
      out << "."; // std::setw(3) << std::setprecision(4) << ".";
    }
    return out;
  }
};
} // namespace mylib
#endif