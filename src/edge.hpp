/**
 * @author Wajih Ouertani
 * @email wajih.ouertani@gmail.com
 */

#ifndef MYLIB_EDGE
#define MYLIB_EDGE

#include <functional>
#include <iomanip>
#include <ostream>

#include "utils.hpp"

namespace mylib {

template <class Node> class Edge {
private:
  bool enabled;
  double value;
  Node from_, to_; // two ends of the edge

public:
  /**
   * Get value
   */
  double get_value() const { return this->value; }

  bool set_value(double value) {
    this->value = value;
    return true;
  }
  /**
   * get edge status
   */
  bool is_enabled() const { return enabled; }
  Node from() const { return this->from_; }
  Node to() const { return this->to_; }

  /**
   * Randomly generate edges
   * returns an edge for each call
   * where the value is between lower and upper bounds
   */
  static std::function<Edge<Node>(Node, Node)>
  generate(double lower = 0., double upper = 1.) {
    return [&](Node from, Node to) {
      double dist = utils::random<double>(lower, upper);
      return Edge<Node>(from, to, dist);
    };
  }

  /**
   * Generate (more than) filter given a threshold
   */
  static std::function<bool(Edge<Node>)> more_than_filter(double threshold) {
    return [&](Edge<Node> e) { return (e.get_value() > threshold); };
  }

  /**
   * Generate (less than) filter given a threshold
   */
  static std::function<bool(Edge<Node>)> less_than_filter(double threshold) {
    return [&](Edge<Node> e) { return (e.get_value() < threshold); };
  }

  /**
   * builds/return reverse (symmetric: same value/undirected graph) Edge from
   * to_ to from_
   */
  Edge<Node> sym_reverse() {
    return  Edge<Node>(this->to_, this->from_, this->enabled,
                          this->value);
  }

  Edge(Node from_, Node to_)
      : enabled(false), value(0.0), from_(from_), to_(to_) {}

  Edge(Node from_, Node to_, double _dist, bool _enabled=true)
      : enabled(_enabled), value(_dist), from_(from_), to_(to_) {}

  bool collapsed() { return value == 0.0; }


  bool &operator==(const Edge &other){
    return ((this->from() == other.from()) && (this->to() == other.to()) && (this->get_value() == other.get_value()));
  }


  friend std::ostream &operator<<(std::ostream &out, const Edge &e) {
    if (e.is_enabled()) {
      out << e.from() << "->" << e.to() << "(" << e.get_value() << ")"; //  << std::setw(3) << std::setprecision(3) <<
                               //  e.get_value();
    } else {
      out << ".";
    }
    return out;
  }
};


template <class Node>
  static bool operator==(const Edge<Node> &e1, const Edge<Node> &e2){
    return ((e1.from() == e2.from()) && (e1.to() == e2.to()) && (e1.get_value() == e2.get_value()));
  }

} // namespace mylib
#endif
