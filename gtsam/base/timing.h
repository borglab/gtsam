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
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/version.hpp>
#include <gtsam/global_includes.h>
#include <gtsam/base/FastMap.h>

// Automatically use the new Boost timers if version is recent enough.
#if BOOST_VERSION >= 104800
#ifndef GTSAM_DISABLE_NEW_TIMERS
#define GTSAM_USING_NEW_BOOST_TIMERS
#endif
#endif

#ifdef GTSAM_USING_NEW_BOOST_TIMERS
#include <boost/timer/timer.hpp>
#else
#include <boost/timer.hpp>
#endif

namespace gtsam {

  namespace internal {
    GTSAM_EXPORT size_t getTicTocID(const char *description);
    GTSAM_EXPORT void ticInternal(size_t id, const char *label);
    GTSAM_EXPORT void tocInternal(size_t id, const char *label);

    class GTSAM_EXPORT TimingOutline {
    protected:
      size_t myId_;
      size_t t_;
      size_t tWall_;
      double t2_ ; /* cache the \sum t_i^2 */
      size_t tIt_;
      size_t tMax_;
      size_t tMin_;
      size_t n_;
      size_t myOrder_;
      size_t lastChildOrder_;
      std::string label_;
      boost::weak_ptr<TimingOutline> parent_;
      typedef FastMap<size_t, boost::shared_ptr<TimingOutline> > ChildMap;
      ChildMap children_;
#ifdef GTSAM_USING_NEW_BOOST_TIMERS
      boost::timer::cpu_timer timer_;
#else
      boost::timer timer_;
      gtsam::ValueWithDefault<bool,false> timerActive_;
#endif
      void add(size_t usecs, size_t usecsWall);
    public:
      TimingOutline(const std::string& label, size_t myId);
      size_t time() const;
      void print(const std::string& outline = "") const;
      void print2(const std::string& outline = "", const double parentTotal = -1.0) const;
      const boost::shared_ptr<TimingOutline>& child(size_t child, const std::string& label, const boost::weak_ptr<TimingOutline>& thisPtr);
      void ticInternal();
      void tocInternal();
      void finishedIteration();

      GTSAM_EXPORT friend void tocInternal(size_t id, const char *label);
    }; // \TimingOutline

    class AutoTicToc {
    private:
      size_t id_;
      const char *label_;
      bool isSet_;
    public:
      AutoTicToc(size_t id, const char* label) : id_(id), label_(label), isSet_(true) { ticInternal(id_, label_); }
      void stop() { tocInternal(id_, label_); isSet_ = false; }
      ~AutoTicToc() { if(isSet_) stop(); }
    };

    GTSAM_EXTERN_EXPORT boost::shared_ptr<TimingOutline> timingRoot;
    GTSAM_EXTERN_EXPORT boost::weak_ptr<TimingOutline> timingCurrent;
  }

// Tic and toc functions that are always active (whether or not ENABLE_TIMING is defined)
// There is a trick being used here to achieve near-zero runtime overhead, in that a
// static variable is created for each tic/toc statement storing an integer ID, but the
// integer ID is only looked up by string once when the static variable is initialized
// as the program starts.
#define gttic_(label) \
  static const size_t label##_id_tic = ::gtsam::internal::getTicTocID(#label); \
  ::gtsam::internal::AutoTicToc label##_obj = ::gtsam::internal::AutoTicToc(label##_id_tic, #label)
#define gttoc_(label) \
  label##_obj.stop()
#define longtic_(label) \
  static const size_t label##_id_tic = ::gtsam::internal::getTicTocID(#label); \
  ::gtsam::internal::ticInternal(label##_id_tic, #label)
#define longtoc_(label) \
  static const size_t label##_id_toc = ::gtsam::internal::getTicTocID(#label); \
  ::gtsam::internal::tocInternal(label##_id_toc, #label)
inline void tictoc_finishedIteration_() {
  internal::timingRoot->finishedIteration(); }
inline void tictoc_print_() {
  internal::timingRoot->print(); }
/* print mean and standard deviation */
inline void tictoc_print2_() {
  internal::timingRoot->print2(); }
#define tictoc_getNode(variable, label) \
  static const size_t label##_id_getnode = ::gtsam::internal::getTicTocID(#label); \
  const boost::shared_ptr<const internal::TimingOutline> variable = \
  internal::timingCurrent.lock()->child(label##_id_getnode, #label, internal::timingCurrent);
inline void tictoc_reset_() {
  internal::timingRoot.reset(new internal::TimingOutline("Total", internal::getTicTocID("Total")));
  internal::timingCurrent = internal::timingRoot; }

#ifdef ENABLE_TIMING
#define gttic(label) gttic_(label)
#define gttoc(label) gttoc_(label)
#define longtic(label) longtic_(label)
#define longtoc(label) longtoc_(label)
#define tictoc_finishedIteration tictoc_finishedIteration_
#define tictoc_print tictoc_print_
#define tictoc_reset tictoc_reset_
#else
#define gttic(label) ((void)0)
#define gttoc(label) ((void)0)
#define longtic(label) ((void)0)
#define longtoc(label) ((void)0)
#define tictoc_finishedIteration() ((void)0)
#define tictoc_print() ((void)0)
#define tictoc_reset() ((void)0)
#endif

}
