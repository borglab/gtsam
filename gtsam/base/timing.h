/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timing.h
 * @brief   
 * @author  Richard Roberts (extracted from Michael Kaess' timing functions)
 * @created Oct 5, 2010
 */
#pragma once

#include <stdio.h>
#include <string>

// simple class for accumulating execution timing information by name
class Timing;
extern Timing timing;

double tic();
double toc(double t);
double tic(const std::string& id);
double toc(const std::string& id);
void tictoc_print();

/** These underscore versions work evening when ENABLE_TIMING is not defined */
double tic_();
double toc_(double t);
double tic_(const std::string& id);
double toc_(const std::string& id);
void tictoc_print_();



#include <sys/time.h>
#include <map>
#include <string>
// simple class for accumulating execution timing information by name
class Timing {
  class Stats {
  public:
    double t0;
    double t;
    double t_max;
    double t_min;
    int n;
  };
  std::map<std::string, Stats> stats;
public:
  void add_t0(const std::string& id, double t0) {
    stats[id].t0 = t0;
  }
  double get_t0(const std::string& id) {
    return stats[id].t0;
  }
  void add_dt(const std::string& id, double dt) {
    Stats& s = stats[id];
    s.t += dt;
    s.n++;
    if (s.n==1 || s.t_max < dt) s.t_max = dt;
    if (s.n==1 || s.t_min > dt) s.t_min = dt;
  }
  void print() {
    std::map<std::string, Stats>::iterator it;
    for(it = stats.begin(); it!=stats.end(); it++) {
      Stats& s = it->second;
      printf("%s: %g (%i times, min: %g, max: %g)\n",
          it->first.c_str(), s.t, s.n, s.t_min, s.t_max);
    }
  }
  double time(const std::string& id) {
    Stats& s = stats[id];
    return s.t;
  }
};

inline double tic_() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
}
inline double toc_(double t) {
  double s = tic_();
  return (std::max(0., s-t));
}
inline double tic_(const std::string& id) {
  double t0 = tic_();
  timing.add_t0(id, t0);
  return t0;
}
inline double toc_(const std::string& id) {
  double dt = toc_(timing.get_t0(id));
  timing.add_dt(id, dt);
  return dt;
}
inline void tictoc_print_() {
  timing.print();
}

#ifdef ENABLE_TIMING
inline double tic() { return tic_(); }
inline double toc(double t) { return toc_(t); }
inline double tic(const std::string& id) { return tic_(id); }
inline double toc(const std::string& id) { return toc_(id); }
inline void tictoc_print() { tictoc_print_(); }
#else
inline double tic() {return 0.;}
inline double toc(double t) {return 0.;}
inline double tic(const std::string& id) {return 0.;}
inline double toc(const std::string& id) {return 0.;}
inline void tictoc_print() {}
#endif
