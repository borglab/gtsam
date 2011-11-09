/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timing.h
 * @brief   Timing utilities
 * @author  Richard Roberts, Michael Kaess
 * @date    Oct 5, 2010
 */
#pragma once

#include <string>
#include <map>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

class TimingOutline;
extern boost::shared_ptr<TimingOutline> timingRoot;
extern boost::weak_ptr<TimingOutline> timingCurrent;

class TimingOutline {
protected:
  size_t t_;
  size_t tIt_;
  size_t tMax_;
  size_t tMin_;
  size_t n_;
  std::string label_;
  boost::weak_ptr<TimingOutline> parent_;
  std::vector<boost::shared_ptr<TimingOutline> > children_;
  struct timeval t0_;
  bool timerActive_;

  void add(size_t usecs);

public:

  TimingOutline(const std::string& label);

  size_t time() const;

  void print(const std::string& outline = "") const;

  const boost::shared_ptr<TimingOutline>& child(size_t child, const std::string& label, const boost::weak_ptr<TimingOutline>& thisPtr);

  void tic();

  void toc();

  void finishedIteration();

  friend class AutoTimer;
  friend void toc_(size_t id);
  friend void toc_(size_t id, const std::string& label);
}; // \TimingOutline

void tic_(size_t id, const std::string& label);

void toc_(size_t id);

void toc_(size_t id, const std::string& label);

inline void tictoc_finishedIteration_() {
  timingRoot->finishedIteration();
}

#ifdef ENABLE_TIMING
inline void tic(size_t id, const std::string& label) { tic_(id, label); }
inline void toc(size_t id) { toc_(id); }
inline void toc(size_t id, const std::string& label) { toc_(id, label); }
inline void tictoc_finishedIteration() { tictoc_finishedIteration_(); }
#else
inline void tic(size_t, const char*) {}
inline void toc(size_t) {}
inline void toc(size_t, const char*) {}
inline void tictoc_finishedIteration() {}
#endif

// simple class for accumulating execution timing information by name
class Timing;
extern Timing timing;
extern std::string timingPrefix;

double _tic();
double _toc(double t);
double tic(const std::string& id);
double toc(const std::string& id);
void ticPush(const std::string& id);
void ticPop(const std::string& id);
void tictoc_print();
void tictoc_finishedIteration();

/** These underscore versions work evening when ENABLE_TIMING is not defined */
double _tic_();
double _toc_(double t);
double tic_(const std::string& id);
double toc_(const std::string& id);
void ticPush_(const std::string& id);
void ticPop_(const std::string& id);
void tictoc_print_();
void tictoc_finishedIteration_();



// simple class for accumulating execution timing information by name
class Timing {
  class Stats {
  public:
    std::string label;
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
  void print();

  double time(const std::string& id) {
    Stats& s = stats[id];
    return s.t;
  }
};

double _tic_();
inline double _toc_(double t) {
  double s = _tic_();
  return (std::max(0., s-t));
}
inline double tic_(const std::string& id) {
  double t0 = _tic_();
  timing.add_t0(timingPrefix + " " + id, t0);
  return t0;
}
inline double toc_(const std::string& id) {
  std::string comb(timingPrefix + " " + id);
  double dt = _toc_(timing.get_t0(comb));
  timing.add_dt(comb, dt);
  return dt;
}
inline void ticPush_(const std::string& prefix, const std::string& id) {
  if(timingPrefix.size() > 0)
    timingPrefix += ".";
  timingPrefix += prefix;
  tic_(id);
}
void ticPop_(const std::string& prefix, const std::string& id);
inline void tictoc_print_() {
  timing.print();
  timingRoot->print();
}

#ifdef ENABLE_TIMING
inline double _tic() { return _tic_(); }
inline double _toc(double t) { return _toc_(t); }
inline double tic(const std::string& id) { return tic_(id); }
inline double toc(const std::string& id) { return toc_(id); }
inline void ticPush(const std::string& prefix, const std::string& id) { ticPush_(prefix, id); }
inline void ticPop(const std::string& prefix, const std::string& id) { ticPop_(prefix, id); }
inline void tictoc_print() { tictoc_print_(); }
#else
inline double _tic() {return 0.;}
inline double _toc(double) {return 0.;}
inline double tic(const std::string&) {return 0.;}
inline double toc(const std::string&) {return 0.;}
inline void ticPush(const std::string&, const std::string&) {}
inline void ticPop(const std::string&, const std::string&) {}
inline void tictoc_print() {}
#endif
