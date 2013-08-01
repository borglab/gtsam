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

#include <math.h>
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <stdlib.h>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>

#include <gtsam/base/debug.h>
#include <gtsam/base/timing.h>

namespace gtsam {

/* ************************************************************************* */
namespace internal {

GTSAM_EXPORT boost::shared_ptr<TimingOutline> timingRoot(new TimingOutline("Total", getTicTocID("Total")));
GTSAM_EXPORT boost::weak_ptr<TimingOutline> timingCurrent(timingRoot);

/* ************************************************************************* */
// Implementation of TimingOutline
/* ************************************************************************* */

/* ************************************************************************* */
void TimingOutline::add(size_t usecs, size_t usecsWall) {
  t_ += usecs;
  tWall_ += usecsWall;
  tIt_ += usecs;
  t2_ += (double(usecs)/1000000.0)*(double(usecs)/1000000.0);
  ++ n_;
}

/* ************************************************************************* */
TimingOutline::TimingOutline(const std::string& label, size_t myId) :
   myId_(myId), t_(0), tWall_(0), t2_(0.0), tIt_(0), tMax_(0), tMin_(0), n_(0), myOrder_(0), lastChildOrder_(0), label_(label)
{
#ifdef GTSAM_USING_NEW_BOOST_TIMERS
  timer_.stop();
#endif
}

/* ************************************************************************* */
size_t TimingOutline::time() const {
  size_t time = 0;
  bool hasChildren = false;
  BOOST_FOREACH(const ChildMap::value_type& child, children_) {
    time += child.second->time();
    hasChildren = true;
  }
  if(hasChildren)
    return time;
  else
    return t_;
}

/* ************************************************************************* */
void TimingOutline::print(const std::string& outline) const {
  std::string formattedLabel = label_;
  boost::replace_all(formattedLabel, "_", " ");
  std::cout << outline << "-" << formattedLabel << ": " << double(t_)/1000000.0 << " CPU (" <<
      n_ << " times, " << double(tWall_)/1000000.0 << " wall, " << double(time())/1000000.0 << " children, min: "
      << double(tMin_)/1000000.0 << " max: " << double(tMax_)/1000000.0 << ")\n";
  // Order children
  typedef FastMap<size_t, boost::shared_ptr<TimingOutline> > ChildOrder;
  ChildOrder childOrder;
  BOOST_FOREACH(const ChildMap::value_type& child, children_) {
    childOrder[child.second->myOrder_] = child.second;
  }
  // Print children
  BOOST_FOREACH(const ChildOrder::value_type order_child, childOrder) {
    std::string childOutline(outline);
    childOutline += "|   ";
    order_child.second->print(childOutline);
  }
  std::cout.flush();
}

void TimingOutline::print2(const std::string& outline, const double parentTotal) const {

  const int w1 = 24, w2 = 2, w3 = 6, w4 = 8, precision = 2;
  const double selfTotal = double(t_)/(1000000.0),
               selfMean = selfTotal/double(n_);
  const double childTotal = double(time())/(1000000.0);

  // compute standard deviation
  const double selfStd = sqrt(t2_/double(n_) - selfMean*selfMean);
  const std::string label = outline + label_ + ": " ;

  if ( n_ == 0 ) {
    std::cout << label << std::fixed << std::setprecision(precision) << childTotal << " seconds" << std::endl;
  }
  else {
    std::cout << std::setw(w1+outline.length()) << label ;
    std::cout << std::setiosflags(std::ios::right) << std::setw(w2) << n_ << " (times), "
      << std::setiosflags(std::ios::right) << std::fixed << std::setw(w3) << std::setprecision(precision) << selfMean << " (mean), "
      << std::setiosflags(std::ios::right) << std::fixed << std::setw(w3) << std::setprecision(precision) << selfStd  << " (std),"
      << std::setiosflags(std::ios::right) << std::fixed << std::setw(w4) << std::setprecision(precision) << selfTotal  << " (total),";

    if ( parentTotal > 0.0 )
      std::cout << std::setiosflags(std::ios::right) << std::fixed << std::setw(w3) << std::setprecision(precision) << 100.0*selfTotal/parentTotal  << " (%)";

    std::cout << std::endl;
  }

  BOOST_FOREACH(const ChildMap::value_type& child, children_) {
    std::string childOutline(outline);
    if ( n_ == 0 ) {
      child.second->print2(childOutline, childTotal);
    }
    else {
      childOutline += "  ";
      child.second->print2(childOutline, selfTotal);
    }
  }
}

/* ************************************************************************* */
const boost::shared_ptr<TimingOutline>& TimingOutline::child(size_t child, const std::string& label, const boost::weak_ptr<TimingOutline>& thisPtr) {
  assert(thisPtr.lock().get() == this);
  boost::shared_ptr<TimingOutline>& result = children_[child];
  if(!result) {
    // Create child if necessary
    result.reset(new TimingOutline(label, child));
    ++ this->lastChildOrder_;
    result->myOrder_ = this->lastChildOrder_;
    result->parent_ = thisPtr;
  }
  return result;
}

/* ************************************************************************* */
void TimingOutline::ticInternal() {
#ifdef GTSAM_USING_NEW_BOOST_TIMERS
  assert(timer_.is_stopped());
  timer_.start();
#else
  assert(!timerActive_);
  timer_.restart();
  *timerActive_ = true;
#endif
}

/* ************************************************************************* */
void TimingOutline::tocInternal() {
#ifdef GTSAM_USING_NEW_BOOST_TIMERS
  assert(!timer_.is_stopped());
  timer_.stop();
  add((timer_.elapsed().user + timer_.elapsed().system) / 1000, timer_.elapsed().wall / 1000);
#else
  assert(timerActive_);
  double elapsed = timer_.elapsed();
  add(size_t(elapsed * 1000000.0), 0);
  *timerActive_ = false;
#endif
}

/* ************************************************************************* */
void TimingOutline::finishedIteration() {
  if(tIt_ > tMax_)
    tMax_ = tIt_;
  if(tMin_ == 0 || tIt_ < tMin_)
    tMin_ = tIt_;
  tIt_ = 0;
  BOOST_FOREACH(ChildMap::value_type& child, children_) {
    child.second->finishedIteration();
  }
}

  /* ************************************************************************* */
  // Generate or retrieve a unique global ID number that will be used to look up tic_/toc statements
  size_t getTicTocID(const char *descriptionC) {
    const std::string description(descriptionC);
    // Global (static) map from strings to ID numbers and current next ID number
    static size_t nextId = 0;
    static gtsam::FastMap<std::string, size_t> idMap;

    // Retrieve or add this string
    gtsam::FastMap<std::string, size_t>::const_iterator it = idMap.find(description);
    if(it == idMap.end()) {
      it = idMap.insert(std::make_pair(description, nextId)).first;
      ++ nextId;
    }

    // Return ID
    return it->second;
  }

  /* ************************************************************************* */
  void ticInternal(size_t id, const char *labelC) {
    const std::string label(labelC);
    if(ISDEBUG("timing-verbose"))
      std::cout << "gttic_(" << id << ", " << label << ")" << std::endl;
    boost::shared_ptr<TimingOutline> node = timingCurrent.lock()->child(id, label, timingCurrent);
    timingCurrent = node;
    node->ticInternal();
  }

  /* ************************************************************************* */
  void tocInternal(size_t id, const char *label) {
    if(ISDEBUG("timing-verbose"))
      std::cout << "gttoc(" << id << ", " << label << ")" << std::endl;
    boost::shared_ptr<TimingOutline> current(timingCurrent.lock());
    if(id != current->myId_) {
      timingRoot->print();
      throw std::invalid_argument(
        (boost::format("gtsam timing:  Mismatched tic/toc: gttoc(\"%s\") called when last tic was \"%s\".") %
        label % current->label_).str());
    }
    if(!current->parent_.lock()) {
      timingRoot->print();
      throw std::invalid_argument(
        (boost::format("gtsam timing:  Mismatched tic/toc: extra gttoc(\"%s\"), already at the root") %
        label).str());
    }
    current->tocInternal();
    timingCurrent = current->parent_;
  }

}

}
