/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timing.cpp
 * @brief   Timing utilities
 * @author  Richard Roberts, Michael Kaess, Frank Dellaert
 * @date    Oct 5, 2010
 */

#include <gtsam/base/debug.h>
#include <gtsam/base/timing.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <map>
#include <stdexcept>
#include <utility>

#if !GTSAM_USE_BOOST_FEATURES
#include <chrono>
#include <ctime>
#endif

namespace gtsam {
namespace internal {

using ChildOrder = FastMap<size_t, std::shared_ptr<TimingOutline>>;

// a static shared_ptr to TimingOutline with nullptr as the pointer
const static std::shared_ptr<TimingOutline> nullTimingOutline;

GTSAM_EXPORT std::shared_ptr<TimingOutline> gTimingRoot(
    new TimingOutline("Total", getTicTocID("Total")));
GTSAM_EXPORT std::weak_ptr<TimingOutline> gCurrentTimer(gTimingRoot);

/* ************************************************************************* */
// Implementation of TimingOutline
/* ************************************************************************* */

/* ************************************************************************* */
void TimingOutline::add(size_t usecs, size_t usecsWall) {
  t_ += usecs;
  tWall_ += usecsWall;
  tIt_ += usecs;
  double secs = (double(usecs) / 1000000.0);
  t2_ += secs * secs;
  ++n_;
}

/* ************************************************************************* */
TimingOutline::TimingOutline(const std::string& label, size_t id) :
    id_(id), t_(0), tWall_(0), t2_(0.0), tIt_(0), tMax_(0), tMin_(0), n_(0), myOrder_(
        0), lastChildOrder_(0), label_(label) {
#if GTSAM_USE_BOOST_FEATURES
#ifdef GTSAM_USING_NEW_BOOST_TIMERS
  timer_.stop();
#endif
#endif
}

/* ************************************************************************* */
size_t TimingOutline::time() const {
  size_t time = 0;
  bool hasChildren = false;
  for(const ChildMap::value_type& child: children_) {
    time += child.second->time();
    hasChildren = true;
  }
  if (hasChildren)
    return time;
  else
    return t_;
}

/* ************************************************************************* */
void TimingOutline::print(const std::string& outline) const {
  std::string formattedLabel = label_;
  std::replace(formattedLabel.begin(), formattedLabel.end(), '_', ' ');
  std::cout << outline << "-" << formattedLabel << ": " << self() << " CPU ("
      << n_ << " times, " << wall() << " wall, " << secs() << " children, min: "
      << min() << " max: " << max() << ")\n";
  // Order children
  ChildOrder childOrder;
  for(const ChildMap::value_type& child: children_) {
    childOrder[child.second->myOrder_] = child.second;
  }
  // Print children
  for(const ChildOrder::value_type& order_child: childOrder) {
    std::string childOutline(outline);
    childOutline += "|   ";
    order_child.second->print(childOutline);
  }
  std::cout.flush();
}

/* ************************************************************************* */
void TimingOutline::printCsvHeader(bool addLineBreak) const {
  // Order is (CPU time, number of times, wall time, time + children in seconds,
  // min time, max time)
  std::cout << label_ + " cpu time (s)" << "," << label_ + " #calls" << ","
            << label_ + " wall time(s)" << "," << label_ + " subtree time (s)"
            << "," << label_ + " min time (s)" << "," << label_ + "max time(s)"
            << ",";
  // Order children
  ChildOrder childOrder;
  for (const ChildMap::value_type& child : children_) {
    childOrder[child.second->myOrder_] = child.second;
  }
  // Print children
  for (const ChildOrder::value_type& order_child : childOrder) {
    order_child.second->printCsvHeader();
  }
  if (addLineBreak) {
    std::cout << std::endl;
  }
  std::cout.flush();
}

/* ************************************************************************* */
void TimingOutline::printCsv(bool addLineBreak) const {
  // Order is (CPU time, number of times, wall time, time + children in seconds,
  // min time, max time)
  std::cout << self() << "," << n_ << "," << wall() << "," << secs() << ","
            << min() << "," << max() << ",";
  // Order children
  ChildOrder childOrder;
  for (const ChildMap::value_type& child : children_) {
    childOrder[child.second->myOrder_] = child.second;
  }
  // Print children
  for (const ChildOrder::value_type& order_child : childOrder) {
    order_child.second->printCsv(false);
  }
  if (addLineBreak) {
    std::cout << std::endl;
  }
  std::cout.flush();
}

void TimingOutline::print2(const std::string& outline,
    const double parentTotal) const {
  const int w1 = 24, w2 = 2, w3 = 6, w4 = 8, precision = 2;
  const double selfTotal = self(), selfMean = (n_ > 0) ? selfTotal / double(n_) : 0.0;
  const double childTotal = secs();

  // compute standard deviation
  const double selfStd = (n_ > 0) ? sqrt(t2_ / double(n_) - selfMean * selfMean) : 0.0;
  const std::string label = outline + label_ + ": ";

  if (n_ == 0) {
    std::cout << label << std::fixed << std::setprecision(precision)
        << childTotal << " seconds" << std::endl;
  } else {
    std::cout << std::setw(w1 + outline.length()) << label;
    std::cout << std::setiosflags(std::ios::right) << std::setw(w2) << n_
        << " (times), " << std::setiosflags(std::ios::right) << std::fixed
        << std::setw(w3) << std::setprecision(precision) << selfMean
        << " (mean), " << std::setiosflags(std::ios::right) << std::fixed
        << std::setw(w3) << std::setprecision(precision) << selfStd << " (std),"
        << std::setiosflags(std::ios::right) << std::fixed << std::setw(w4)
        << std::setprecision(precision) << selfTotal << " (total),";

    if (parentTotal > 0.0)
      std::cout << std::setiosflags(std::ios::right) << std::fixed
          << std::setw(w3) << std::setprecision(precision)
          << 100.0 * selfTotal / parentTotal << " (%)";

    std::cout << std::endl;
  }

  for(const ChildMap::value_type& child: children_) {
    std::string childOutline(outline);
    if (n_ == 0) {
      child.second->print2(childOutline, childTotal);
    } else {
      childOutline += "  ";
      child.second->print2(childOutline, selfTotal);
    }
  }
}

/* ************************************************************************* */
const std::shared_ptr<TimingOutline>& TimingOutline::child(size_t child,
    const std::string& label, const std::weak_ptr<TimingOutline>& thisPtr) {
#if GTSAM_USE_BOOST_FEATURES
  assert(thisPtr.lock().get() == this);
  std::shared_ptr<TimingOutline>& result = children_[child];
  if (!result) {
    // Create child if necessary
    result.reset(new TimingOutline(label, child));
    ++this->lastChildOrder_;
    result->myOrder_ = this->lastChildOrder_;
    result->parent_ = thisPtr;
  }
  return result;
#else
  assert(thisPtr.lock().get() == this);
  std::shared_ptr<TimingOutline>& result = children_[child];
  if (!result) {
    // Create child if necessary
    result.reset(new TimingOutline(label, child));
    ++this->lastChildOrder_;
    result->myOrder_ = this->lastChildOrder_;
    result->parent_ = thisPtr;
  }
  return result;
#endif
}

/* ************************************************************************* */
void TimingOutline::tic() {
#if GTSAM_USE_BOOST_FEATURES
#ifdef GTSAM_USING_NEW_BOOST_TIMERS
  assert(timer_.is_stopped());
  timer_.start();
#else
  assert(!timerActive_);
  timer_.restart();
  *timerActive_ = true;
#endif

#ifdef GTSAM_USE_TBB
  tbbTimer_ = tbb::tick_count::now();
#endif
#else  // GTSAM_USE_BOOST_FEATURES
  assert(!timer_active_);
  cpu_timer_start_ = std::clock();
  wall_timer_start_ = std::chrono::steady_clock::now();
  timer_active_ = true;
#endif // GTSAM_USE_BOOST_FEATURES
}

/* ************************************************************************* */
void TimingOutline::toc() {
#if GTSAM_USE_BOOST_FEATURES

#ifdef GTSAM_USING_NEW_BOOST_TIMERS

  assert(!timer_.is_stopped());
  timer_.stop();
  size_t cpuTime = (timer_.elapsed().user + timer_.elapsed().system) / 1000;
#  ifndef GTSAM_USE_TBB
  size_t wallTime = timer_.elapsed().wall / 1000;
#  endif

#else

  assert(timerActive_);
  double elapsed = timer_.elapsed();
  size_t cpuTime = size_t(elapsed * 1000000.0);
  *timerActive_ = false;
#  ifndef GTSAM_USE_TBB
  size_t wallTime = cpuTime;
#  endif

#endif

#ifdef GTSAM_USE_TBB
  size_t wallTime = size_t(
      (tbb::tick_count::now() - tbbTimer_).seconds() * 1e6);
#endif

  add(cpuTime, wallTime);
#else // GTSAM_USE_BOOST_FEATURES
  assert(timer_active_);

  // measure CPU time
  const std::clock_t cpu_end = std::clock();
  const double cpu_secs = static_cast<double>(cpu_end - cpu_timer_start_) / CLOCKS_PER_SEC;
  const size_t cpuTime = static_cast<size_t>(cpu_secs * 1000000.0);

  // measure wall time
  const auto wall_end = std::chrono::steady_clock::now();
  const size_t wallTime =
      std::chrono::duration_cast<std::chrono::microseconds>(
          wall_end - wall_timer_start_)
          .count();

  add(cpuTime, wallTime);
  timer_active_ = false;
#endif // GTSAM_USE_BOOST_FEATURES
}

/* ************************************************************************* */
void TimingOutline::finishedIteration() {
  if (tIt_ > tMax_)
    tMax_ = tIt_;
  if (tMin_ == 0 || tIt_ < tMin_)
    tMin_ = tIt_;
  tIt_ = 0;
  for(ChildMap::value_type& child: children_) {
    child.second->finishedIteration();
  }
}

/* ************************************************************************* */
size_t getTicTocID(const char *descriptionC) {
  const std::string description(descriptionC);
  // Global (static) map from strings to ID numbers and current next ID number
  static size_t nextId = 0;
  static gtsam::FastMap<std::string, size_t> idMap;

  // Retrieve or add this string
  auto it = idMap.find(description);
  if (it == idMap.end()) {
    it = idMap.insert({description, nextId}).first;
    ++nextId;
  }

  // Return ID
  return it->second;
}

/* ************************************************************************* */
void tic(size_t id, const char *labelC) {
  const std::string label(labelC);
  std::shared_ptr<TimingOutline> node = //
      gCurrentTimer.lock()->child(id, label, gCurrentTimer);
  gCurrentTimer = node;
  node->tic();
}

/* ************************************************************************* */
void toc(size_t id, const char *labelC) {
  const std::string label(labelC);
  std::shared_ptr<TimingOutline> current(gCurrentTimer.lock());
  if (id != current->id_) {
    gTimingRoot->print();
    throw std::invalid_argument(
        "gtsam timing:  Mismatched tic/toc: gttoc(\"" + label +
        "\") called when last tic was \"" + current->label_ + "\".");
  }
  if (!current->parent_.lock()) {
    gTimingRoot->print();
    throw std::invalid_argument(
        "gtsam timing:  Mismatched tic/toc: extra gttoc(\"" + label +
        "\"), already at the root");
  }
  current->toc();
  gCurrentTimer = current->parent_;
}

} // namespace internal
} // namespace gtsam