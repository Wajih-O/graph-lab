#include <functional>
#include <random>
#include <ostream>
#include <iomanip>

namespace mylib {
/**
 * src: https://stackoverflow.com/questions/2704521/generate-random-double-numbers-in-c
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

class Edge {
private:
  bool enabled;
  double distance;

public:
  double get_distance() const { return distance; }
  bool is_enabled() const { return enabled; }
  /**
   * Randomely generate edges
   * returns an edge for each call
   * where the distance is between lower and upper bounds
   */

  static std::function<Edge()> generate(double lower = 0., double upper = 1.) {
    return [&]() {
      double dist = random<double>(lower, upper);
      return Edge(true, dist);
    };
  }

  /**
   * Generate (more than) filter given a threshold
   */
  static std::function<bool(Edge)> more_than_filter(double threshold) {
    return [&](Edge e) { return (e.get_distance() > threshold); };
  }

  /**
   * Generate (less than) filter given a threshold
   */
  static std::function<bool(Edge)> less_than_filter(double threshold) {
    return [&](Edge e) { return (e.get_distance() < threshold); };
  }

  Edge() : enabled(false), distance(0.0) {}
 
  Edge(bool _enabled, double _dist) : enabled(_enabled), distance(_dist) {}
 
  bool collapsed() { return distance == 0.0; }

  friend std::ostream &operator<<(std::ostream &out, const Edge &e) {
    if (e.is_enabled()) {
      out << std::setw(5) << std::setprecision(3) << e.get_distance();
    } else {
      out << std::setw(5) << std::setprecision(4) << ".";
    }
    return out;
  }
};
}
