#ifndef UTILS
#define UTILS

#include <functional>
#include <iomanip>
#include <ostream>
#include <random>

namespace utils {
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
}
#endif // UTILS