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
 * @date 		Oct 5, 2010
 */

#include <math.h>
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <stdlib.h>
#include <boost/foreach.hpp>
#include <boost/format.hpp>

#include <gtsam/base/debug.h>
#include <gtsam/base/timing.h>

boost::shared_ptr<TimingOutline> timingRoot(new TimingOutline("Total"));
boost::weak_ptr<TimingOutline> timingCurrent(timingRoot);

#ifdef ENABLE_OLD_TIMING
Timing timing;
#endif
std::string timingPrefix;

/* ************************************************************************* */
// Implementation of TimingOutline
/* ************************************************************************* */

/* ************************************************************************* */
void TimingOutline::add(size_t usecs) {
	t_ += usecs;
	tIt_ += usecs;
	t2_ += (double(usecs)/1000000.0)*(double(usecs)/1000000.0);
	++ n_;
}

/* ************************************************************************* */
TimingOutline::TimingOutline(const std::string& label) :
   t_(0), t2_(0.0), tIt_(0), tMax_(0), tMin_(0), n_(0), label_(label) {}

/* ************************************************************************* */
size_t TimingOutline::time() const {
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

/* ************************************************************************* */
void TimingOutline::print(const std::string& outline) const {
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

  for(size_t i=0; i<children_.size(); ++i) {
    if(children_[i]) {
      std::string childOutline(outline);
      if ( n_ == 0 ) {
        children_[i]->print2(childOutline, childTotal);
      }
      else {
        childOutline += "  ";
        children_[i]->print2(childOutline, selfTotal);
      }
    }
  }
}

/* ************************************************************************* */
const boost::shared_ptr<TimingOutline>& TimingOutline::child(size_t child, const std::string& label, const boost::weak_ptr<TimingOutline>& thisPtr) {
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
}

/* ************************************************************************* */
void TimingOutline::toc() {
#ifdef GTSAM_USING_NEW_BOOST_TIMERS
	assert(!timer_.is_stopped());
  timer_.stop();
	add((timer_.elapsed().user + timer_.elapsed().system) / 1000);
#else
  assert(timerActive_);
  double elapsed = timer_.elapsed();
  add(size_t(elapsed * 1000000.0));
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
	for(size_t i=0; i<children_.size(); ++i)
		if(children_[i])
			children_[i]->finishedIteration();
}

/* ************************************************************************* */
void tic_(size_t id, const std::string& label) {
  if(ISDEBUG("timing-verbose"))
    std::cout << "tic(" << id << ", " << label << ")" << std::endl;
  boost::shared_ptr<TimingOutline> node = timingCurrent.lock()->child(id, label, timingCurrent);
  timingCurrent = node;
  node->tic();
}

/* ************************************************************************* */
void toc_(size_t id) {
	if(ISDEBUG("timing-verbose"))
		std::cout << "toc(" << id << ")" << std::endl;
	boost::shared_ptr<TimingOutline> current(timingCurrent.lock());
	if(!(id < current->parent_.lock()->children_.size() && current->parent_.lock()->children_[id] == current)) {
		if(std::find(current->parent_.lock()->children_.begin(), current->parent_.lock()->children_.end(), current)
		!= current->parent_.lock()->children_.end())
			std::cout << "gtsam timing:  Incorrect ID passed to toc, expected "
			<< std::find(current->parent_.lock()->children_.begin(), current->parent_.lock()->children_.end(), current) - current->parent_.lock()->children_.begin()
			<< " \"" << current->label_ << "\", got " << id << std::endl;
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

/* ************************************************************************* */
void toc_(size_t id, const std::string& label) {
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

#ifdef ENABLE_OLD_TIMING

/* ************************************************************************* */
// Timing class implementation
void Timing::print() {
	std::map<std::string, Timing::Stats>::iterator it;
	for(it = this->stats.begin(); it!=stats.end(); it++) {
		Stats& s = it->second;
		printf("%s: %g (%i times, min: %g, max: %g)\n",
				it->first.c_str(), s.t, s.n, s.t_min, s.t_max);
	}
}

/* ************************************************************************* */
double _tic_() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
}

/* ************************************************************************* */
void ticPop_(const std::string& prefix, const std::string& id) {
  toc_(id);
  if(timingPrefix.size() < prefix.size()) {
    fprintf(stderr, "Seems to be a mismatched push/pop in timing, exiting\n");
    exit(1);
  } else if(timingPrefix.size() == prefix.size())
    timingPrefix.resize(0);
  else
    timingPrefix.resize(timingPrefix.size() - prefix.size() - 1);
}

#endif
