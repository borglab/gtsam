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
 * @author  Richard Roberts, Michael Kaess
 * @date     Oct 5, 2010
 */

#include <gtsam/base/debug.h>
#include <gtsam/base/timing.h>

#include <cmath>
#include <cstddef>
#include <cassert>
#include <iomanip>
#include <iostream>
#include <map>
#include <stdexcept>
#include <utility>

namespace gtsam {
namespace internal {

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
#ifdef GTSAM_USING_NEW_BOOST_TIMERS
  timer_.stop();
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
  typedef FastMap<size_t, std::shared_ptr<TimingOutline> > ChildOrder;
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

void TimingOutline::print2(const std::string& outline,
    const double parentTotal) const {

  const int w1 = 24, w2 = 2, w3 = 6, w4 = 8, precision = 2;
  const double selfTotal = self(), selfMean = selfTotal / double(n_);
  const double childTotal = secs();

  // compute standard deviation
  const double selfStd = sqrt(t2_ / double(n_) - selfMean * selfMean);
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
}

/* ************************************************************************* */
void TimingOutline::tic() {
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
}

/* ************************************************************************* */
void TimingOutline::toc() {
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
  gtsam::FastMap<std::string, size_t>::const_iterator it = idMap.find(
      description);
  if (it == idMap.end()) {
    it = idMap.insert(std::make_pair(description, nextId)).first;
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
void toc(size_t id, const char *label) {
  std::shared_ptr<TimingOutline> current(gCurrentTimer.lock());
  if (id != current->id_) {
    gTimingRoot->print();
    throw std::invalid_argument(
        "gtsam timing:  Mismatched tic/toc: gttoc(\"" + std::string(label) +
        "\") called when last tic was \"" + current->label_ + "\".");
  }
  if (!current->parent_.lock()) {
    gTimingRoot->print();
    throw std::invalid_argument(
        "gtsam timing:  Mismatched tic/toc: extra gttoc(\"" + std::string(label) +
        "\"), already at the root");
  }
  current->toc();
  gCurrentTimer = current->parent_;
}

} // namespace internal
} // namespace gtsam
