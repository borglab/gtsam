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
 * @author  Richard Roberts, Michael Kaess
 * @created Oct 5, 2010
 */
#pragma once

#include <gtsam/base/debug.h>
#include <stdio.h>
#include <string>
#include <iostream>
#include <sys/time.h>
#include <map>
#include <stdlib.h>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>


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

  inline void add(size_t usecs) {
    t_ += usecs;
    tIt_ += usecs;
    ++ n_;
  }

public:
  inline TimingOutline(const std::string& label) :
    t_(0), tIt_(0), tMax_(0), tMin_(0), n_(0), label_(label), timerActive_(false) {}

  inline size_t time() const {
    size_t time = 0;
    bool hasChildren = false;
    BOOST_FOREACH(const boost::shared_ptr<TimingOutline>& child, children_) {
      if(child) {
        time += child->time();
        hasChildren = true;
      }
    }
    if(hasChildren)
      return time;
    else
      return t_;
  }

  inline void print(const std::string& outline = "") const {
    std::cout << outline << " " << label_ << ": " << double(t_)/1000000.0 << " (" <<
        n_ << " times, " << double(time())/1000000.0 << " children, min: " << double(tMin_)/1000000.0 <<
        " max: " << double(tMax_)/1000000.0 << ")\n";
    for(size_t i=0; i<children_.size(); ++i) {
      if(children_[i]) {
        std::string childOutline(outline);
        #if 0
        if(childOutline.size() > 0)
          childOutline += ".";
        childOutline += (boost::format("%d") % i).str();
        #else
        childOutline += "  ";
        #endif
        children_[i]->print(childOutline);
      }
    }
  }

  inline const boost::shared_ptr<TimingOutline>& child(size_t child, const std::string& label, const boost::weak_ptr<TimingOutline>& thisPtr) {
    assert(thisPtr.lock().get() == this);
    // Resize if necessary
    if(child >= children_.size())
      children_.resize(child + 1);
    // Create child if necessary
    if(children_[child]) {
#ifndef NDEBUG
      if(children_[child]->label_ != label) {
        timingRoot->print();
        std::cerr << "gtsam timing:  tic called with id=" << child << ", label=" << label << ", but this id already has the label " << children_[child]->label_ << std::endl;
        exit(1);
      }
#endif
    } else {
      children_[child].reset(new TimingOutline(label));
      children_[child]->parent_ = thisPtr;
    }
    return children_[child];
  }

  inline void tic() {
    assert(!timerActive_);
    timerActive_ = true;
    gettimeofday(&t0_, NULL);
  }

  inline void toc() {
    struct timeval t;
    gettimeofday(&t, NULL);
    assert(timerActive_);
    add(t.tv_sec*1000000 + t.tv_usec - (t0_.tv_sec*1000000 + t0_.tv_usec));
    timerActive_ = false;
  }

  inline void finishedIteration() {
    if(tIt_ > tMax_)
      tMax_ = tIt_;
    if(tMin_ == 0 || tIt_ < tMin_)
      tMin_ = tIt_;
    tIt_ = 0;
    for(size_t i=0; i<children_.size(); ++i)
      if(children_[i])
        children_[i]->finishedIteration();
  }

  friend class AutoTimer;
  friend void toc_(size_t id);
  friend void toc_(size_t id, const std::string& label);
};

inline void tic_(size_t id, const std::string& label) {
  if(ISDEBUG("timing-verbose"))
    std::cout << "tic(" << id << ", " << label << ")" << std::endl;
  boost::shared_ptr<TimingOutline> node = timingCurrent.lock()->child(id, label, timingCurrent);
  timingCurrent = node;
  node->tic();
}

inline void toc_(size_t id) {
  if(ISDEBUG("timing-verbose"))
    std::cout << "toc(" << id << ")" << std::endl;
  boost::shared_ptr<TimingOutline> current(timingCurrent.lock());
  if(!(id < current->parent_.lock()->children_.size() && current->parent_.lock()->children_[id] == current)) {
    if(std::find(current->parent_.lock()->children_.begin(), current->parent_.lock()->children_.end(), current)
        != current->parent_.lock()->children_.end())
      std::cout << "gtsam timing:  Incorrect ID passed to toc, expected "
          << std::find(current->parent_.lock()->children_.begin(), current->parent_.lock()->children_.end(), current) - current->parent_.lock()->children_.begin()
          << ", got " << id << std::endl;
    else
      std::cout << "gtsam timing:  Incorrect ID passed to toc, id " << id << " does not exist" << std::endl;
    timingRoot->print();
    throw std::invalid_argument("gtsam timing:  Incorrect ID passed to toc");
  }
  current->toc();
  if(!current->parent_.lock()) {
    std::cout << "gtsam timing:  extra toc, already at the root" << std::endl;
    timingRoot->print();
    throw std::invalid_argument("gtsam timing:  extra toc, already at the root");
  }
  timingCurrent = current->parent_;
}

inline void toc_(size_t id, const std::string& label) {
  if(ISDEBUG("timing-verbose"))
    std::cout << "toc(" << id << ", " << label << ")" << std::endl;
  bool check = false;
#ifndef NDEBUG
  // If NDEBUG is defined, still do this debug check if the granular debugging
  // flag is enabled.  If NDEBUG is not defined, always do this check.
  check = true;
#endif
  if(check || ISDEBUG("timing-debug")) {
    if(label != timingCurrent.lock()->label_) {
      std::cerr << "gtsam timing:  toc called with id=" << id << ", label=\"" << label << "\", but expecting \"" << timingCurrent.lock()->label_ << "\"" << std::endl;
      timingRoot->print();
      exit(1);
    }
  }
  toc_(id);
}

inline void tictoc_finishedIteration_() {
  timingRoot->finishedIteration();
}

#ifdef ENABLE_TIMING
inline void tic(size_t id, const std::string& label) { tic_(id, label); }
inline void toc(size_t id) { toc_(id); }
inline void toc(size_t id, const std::string& label) { toc_(id, label); }
inline void tictoc_finishedIteration() { tictoc_finishedIteration_(); }
#else
inline void tic(size_t id, const std::string& label) {}
inline void toc(size_t id) {}
inline void toc(size_t id, const std::string& label) {}
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

inline double _tic_() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
}
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
inline void ticPop_(const std::string& prefix, const std::string& id) {
  toc_(id);
  if(timingPrefix.size() < prefix.size()) {
    fprintf(stderr, "Seems to be a mismatched push/pop in timing, exiting\n");
    exit(1);
  } else if(timingPrefix.size() == prefix.size())
    timingPrefix.resize(0);
  else
    timingPrefix.resize(timingPrefix.size() - prefix.size() - 1);
}
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
inline double _toc(double t) {return 0.;}
inline double tic(const std::string& id) {return 0.;}
inline double toc(const std::string& id) {return 0.;}
inline void ticPush(const std::string& prefix, const std::string& id) {}
inline void ticPop(const std::string& prefix, const std::string& id) {}
inline void tictoc_print() {}
#endif
