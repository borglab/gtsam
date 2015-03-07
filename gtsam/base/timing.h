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
#include <gtsam/base/types.h>
#include <gtsam/base/FastMap.h>

// Enabling the new Boost timers introduces dependencies on other Boost
// libraries so this is disabled for now until we modify the build scripts
// to link each component or MATLAB wrapper with only the libraries it needs.
#if BOOST_VERSION >= 104800
#define GTSAM_USING_NEW_BOOST_TIMERS
#endif

#ifdef GTSAM_USING_NEW_BOOST_TIMERS
#include <boost/timer/timer.hpp>
#else
#include <boost/timer.hpp>
#endif

namespace gtsam {

  namespace internal {
    size_t getTicTocID(const char *description);
    void ticInternal(size_t id, const char *label);
    void tocInternal(size_t id, const char *label);

    class TimingOutline {
    protected:
      size_t myId_;
      size_t t_;
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
      void add(size_t usecs);
    public:
      TimingOutline(const std::string& label, size_t myId);
      size_t time() const;
      void print(const std::string& outline = "") const;
      void print2(const std::string& outline = "", const double parentTotal = -1.0) const;
      const boost::shared_ptr<TimingOutline>& child(size_t child, const std::string& label, const boost::weak_ptr<TimingOutline>& thisPtr);
      void ticInternal();
      void tocInternal();
      void finishedIteration();

      friend void tocInternal(size_t id);
      friend void tocInternal(size_t id, const char *label);
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

    extern boost::shared_ptr<TimingOutline> timingRoot;
    extern boost::weak_ptr<TimingOutline> timingCurrent;
  }

inline void tictoc_finishedIteration_() {
  internal::timingRoot->finishedIteration();
}

inline void tictoc_print_() {
  internal::timingRoot->print();
}

/* print mean and standard deviation */
inline void tictoc_print2_() {
  internal::timingRoot->print2();
}

// Tic and toc functions using a string label
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

#ifdef ENABLE_TIMING
#define gttic(label) gttic_(label)
#define gttoc(label) gttoc_(label)
#define longtic(label) longtic_(label)
#define longtoc(label) longtoc_(label)
#define tictoc_finishedIteration tictoc_finishedIteration_
#define tictoc_print tictoc_print_
#else
#define gttic(label) ((void)0)
#define gttoc(label) ((void)0)
#define longtic(label) ((void)0)
#define longtoc(label) ((void)0)
inline void tictoc_finishedIteration() {}
inline void tictoc_print() {}
#endif

}