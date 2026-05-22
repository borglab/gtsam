/** This small file provides a pair of convenience functions that are useful for
 * measuring elapsed computation times.
 *
 * Copyright (C) 2017 - 2018 by David M. Rosen (dmrosen@mit.edu)
 */

#pragma once

#include <chrono>

namespace Stopwatch {

/** This function returns a chrono::time_point struct encoding the time at which
 * it is called.*/
inline std::chrono::time_point<std::chrono::high_resolution_clock> tick() {
  return std::chrono::high_resolution_clock::now();
}

/** When this function is called with a chrono::time_point struct returned by
 * tick(), it returns the elapsed time (in seconds) between the calls to tick()
 * and tock().*/
inline double
tock(const std::chrono::time_point<std::chrono::high_resolution_clock>
         &tick_time) {
  auto counter = std::chrono::high_resolution_clock::now() - tick_time;
  return std::chrono::duration_cast<std::chrono::milliseconds>(counter)
             .count() /
         1000.0;
}
} // namespace Stopwatch
