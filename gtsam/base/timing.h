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

#include <gtsam/base/FastMap.h>
#include <gtsam/dllexport.h>
#include <gtsam/config.h> // for GTSAM_USE_TBB

#include <boost/version.hpp>

#include <memory>
#include <cstddef>
#include <string>

// This file contains the GTSAM timing instrumentation library, a low-overhead method for
// learning at a medium-fine level how much time various components of an algorithm take
// in CPU and wall time.
//
// The output of this instrumentation is a call-tree-like printout containing statistics
// about each instrumented code block.  To print this output at any time, call
// tictoc_print() or tictoc_print_().
//
// An overall point to be aware of is that there are two versions of each function - one
// ending in an underscore '_' and one without the trailing underscore.  The underscore
// versions always are active, but the versions without an underscore are active only when
// GTSAM_ENABLE_TIMING is defined (automatically defined in our CMake Timing build type).
// GTSAM algorithms are all instrumented with the non-underscore versions, so generally
// you should use the underscore versions in your own code to leave out the GTSAM detail.
//
// gttic and gttoc start and stop a timed section, respectively.  gttic creates a *scoped*
// object - when it goes out of scope gttoc is called automatically.  Thus, you do not
// need to call gttoc if you are timing an entire function (see basic use examples below).
// However, you must be *aware* of this scoped nature - putting gttic inside of an if(...)
// block, for example, will only time code until the closing brace '}'.  See advanced
// usage below if you need to avoid this.
//
// Multiple calls nest automatically - each gttic nests under the previous gttic called
// for which gttoc has not been called (or the previous gttic did not go out of scope).
//
// Basic usage examples are as follows:
//
// - Timing an entire function:
//   void myFunction() {
//     gttic_(myFunction);
//     ........
//   }
//
// - Timing an entire function as well as its component parts:
//   void myLongFunction() {
//     gttic_(myLongFunction);
//     gttic_(step1); // Will nest under the 'myLongFunction' label
//     ........
//     gttoc_(step1);
//     gttic_(step2); // Will nest under the 'myLongFunction' label
//     ........
//     gttoc_(step2);
//     ........
//   }
//
// - Timing functions calling/called by other functions:
//   void oneStep() {
//     gttic_(oneStep); // Will automatically nest under the gttic label of the calling function
//     .......
//   }
//   void algorithm() {
//     gttic_(algorithm);
//     oneStep(); // gttic's inside this function will automatically nest inside our 'algorithm' label
//     twoStep(); // gttic's inside this function will automatically nest inside our 'algorithm' label
//   }
//
//
// Advanced usage:
//
// - "Finishing iterations" - to get correct min/max times for each call, you must define
//   in your code what constitutes an iteration.  A single sum for the min/max times is
//   accumulated within each iteration.  If you don't care about min/max times, you don't
//   need to worry about this.  For example:
//   void myOuterLoop() {
//     while(true) {
//       iterateMyAlgorithm();
//       tictoc_finishedIteration_();
//       tictoc_print_(); // Optional
//     }
//   }
//
// - Stopping timing a section in a different scope than it is started.  Normally, a gttoc
//   statement goes out of scope at end of C++ scope.  However, you can use longtic and
//   longtoc to start and stop timing with the specified label at any point, without regard
//   too scope.  Note that if you use these, it may become difficult to ensure that you
//   have matching gttic/gttoc statments.  You may want to consider reorganizing your timing
//   outline to match the scope of your code.

// Automatically use the new Boost timers if version is recent enough.
#if BOOST_VERSION >= 104800
#  ifndef GTSAM_DISABLE_NEW_TIMERS
#    define GTSAM_USING_NEW_BOOST_TIMERS
#  endif
#endif

#ifdef GTSAM_USING_NEW_BOOST_TIMERS
#  include <boost/timer/timer.hpp>
#else
#  include <boost/timer.hpp>
#  include <gtsam/base/types.h>
#endif

#ifdef GTSAM_USE_TBB
#  include <tbb/tick_count.h>
#  undef min
#  undef max
#  undef ERROR
#endif

namespace gtsam {

  namespace internal {
    // Generate/retrieve a unique global ID number that will be used to look up tic/toc statements
    GTSAM_EXPORT size_t getTicTocID(const char *description);

    // Create new TimingOutline child for gCurrentTimer, make it gCurrentTimer, and call tic method
    GTSAM_EXPORT void tic(size_t id, const char *label);

    // Call toc on gCurrentTimer and then set gCurrentTimer to the parent of gCurrentTimer
    GTSAM_EXPORT void toc(size_t id, const char *label);

    /**
     * Timing Entry, arranged in a tree
     */
    class TimingOutline {
    protected:
      size_t id_;
      size_t t_;
      size_t tWall_;
      double t2_ ; ///< cache the \f$ \sum t_i^2 \f$
      size_t tIt_;
      size_t tMax_;
      size_t tMin_;
      size_t n_;
      size_t myOrder_;
      size_t lastChildOrder_;
      std::string label_;

      // Tree structure
      std::weak_ptr<TimingOutline> parent_; ///< parent pointer
      typedef FastMap<size_t, std::shared_ptr<TimingOutline> > ChildMap;
      ChildMap children_; ///< subtrees

#ifdef GTSAM_USING_NEW_BOOST_TIMERS
      boost::timer::cpu_timer timer_;
#else
      boost::timer timer_;
      gtsam::ValueWithDefault<bool,false> timerActive_;
#endif
#ifdef GTSAM_USE_TBB
      tbb::tick_count tbbTimer_;
#endif
      void add(size_t usecs, size_t usecsWall);

    public:
      /// Constructor
      GTSAM_EXPORT TimingOutline(const std::string& label, size_t myId);
      GTSAM_EXPORT size_t time() const; ///< time taken, including children
      double secs() const { return double(time()) / 1000000.0;} ///< time taken, in seconds, including children
      double self() const { return double(t_)     / 1000000.0;} ///< self time only, in seconds
      double wall() const { return double(tWall_) / 1000000.0;} ///< wall time, in seconds
      double min()  const { return double(tMin_)  / 1000000.0;} ///< min time, in seconds
      double max()  const { return double(tMax_)  / 1000000.0;} ///< max time, in seconds
      double mean() const { return self() / double(n_); } ///< mean self time, in seconds
      GTSAM_EXPORT void print(const std::string& outline = "") const;
      GTSAM_EXPORT void print2(const std::string& outline = "", const double parentTotal = -1.0) const;
      GTSAM_EXPORT const std::shared_ptr<TimingOutline>&
        child(size_t child, const std::string& label, const std::weak_ptr<TimingOutline>& thisPtr);
      GTSAM_EXPORT void tic();
      GTSAM_EXPORT void toc();
      GTSAM_EXPORT void finishedIteration();

      GTSAM_EXPORT friend void toc(size_t id, const char *label);
    }; // \TimingOutline

    /**
     * Small class that calls internal::tic at construction, and internol::toc when destroyed
     */
    class GTSAM_EXPORT AutoTicToc {
     private:
      size_t id_;
      const char* label_;
      bool isSet_;

     public:
      AutoTicToc(size_t id, const char* label)
          : id_(id), label_(label), isSet_(true) {
        tic(id_, label_);
      }
      void stop() {
        toc(id_, label_);
        isSet_ = false;
      }
      ~AutoTicToc() {
        if (isSet_) stop();
      }
    };

    GTSAM_EXTERN_EXPORT std::shared_ptr<TimingOutline> gTimingRoot;
    GTSAM_EXTERN_EXPORT std::weak_ptr<TimingOutline> gCurrentTimer;
  }

// Tic and toc functions that are always active (whether or not ENABLE_TIMING is defined)
// There is a trick being used here to achieve near-zero runtime overhead, in that a
// static variable is created for each tic/toc statement storing an integer ID, but the
// integer ID is only looked up by string once when the static variable is initialized
// as the program starts.

// tic
#define gttic_(label) \
  static const size_t label##_id_tic = ::gtsam::internal::getTicTocID(#label); \
  ::gtsam::internal::AutoTicToc label##_obj(label##_id_tic, #label)

// toc
#define gttoc_(label) \
  label##_obj.stop()

// tic
#define longtic_(label) \
  static const size_t label##_id_tic = ::gtsam::internal::getTicTocID(#label); \
  ::gtsam::internal::ticInternal(label##_id_tic, #label)

// toc
#define longtoc_(label) \
  static const size_t label##_id_toc = ::gtsam::internal::getTicTocID(#label); \
  ::gtsam::internal::tocInternal(label##_id_toc, #label)

// indicate iteration is finished
inline void tictoc_finishedIteration_() {
  ::gtsam::internal::gTimingRoot->finishedIteration(); }

// print
inline void tictoc_print_() {
  ::gtsam::internal::gTimingRoot->print(); }

// print mean and standard deviation
inline void tictoc_print2_() {
  ::gtsam::internal::gTimingRoot->print2(); }

// get a node by label and assign it to variable
#define tictoc_getNode(variable, label) \
  static const size_t label##_id_getnode = ::gtsam::internal::getTicTocID(#label); \
  const std::shared_ptr<const ::gtsam::internal::TimingOutline> variable = \
  ::gtsam::internal::gCurrentTimer.lock()->child(label##_id_getnode, #label, ::gtsam::internal::gCurrentTimer);

// reset
inline void tictoc_reset_() {
  ::gtsam::internal::gTimingRoot.reset(new ::gtsam::internal::TimingOutline("Total", ::gtsam::internal::getTicTocID("Total")));
  ::gtsam::internal::gCurrentTimer = ::gtsam::internal::gTimingRoot; }

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
