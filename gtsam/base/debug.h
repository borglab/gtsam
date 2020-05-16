/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    debug.h
 * @brief   Global debugging flags
 * @author  Richard Roberts
 * @date    Feb 1, 2011
 */

#include <gtsam/base/FastMap.h>
#include <gtsam/global_includes.h>
#include <string>


// This file defines granular debugging flags that may be switched on and off
// at run time.  Typical usage is 'if(ISDEBUG("myFunction"))' to check if the
// 'myFunction' flag is enabled, and SETDEBUG("myFunction", true) to enable
// this flag, or SETDEBUG("myFunction", false) to disable it.
//
// Debug flags are created automatically as they are accessed, so they can be
// used immediately without explicitly creating them.  Each flag defaults to
// 'false', i.e. disabled.
//
// For these macro to have any effect, granular debugging must be enabled by
// defining GTSAM_ENABLE_DEBUG.  If NDEBUG is not defined, then
// GTSAM_ENABLE_DEBUG will be automatically defined and thus granular
// debugging enabled.

#ifndef NDEBUG
#ifndef GTSAM_ENABLE_DEBUG
#define GTSAM_ENABLE_DEBUG
#endif
#endif

namespace gtsam {
  GTSAM_EXTERN_EXPORT FastMap<std::string, ValueWithDefault<bool,false> > debugFlags;

  // Non-guarded use led to crashes, and solved in commit cd35db2
  bool GTSAM_EXPORT guardedIsDebug(const std::string& s);
  void GTSAM_EXPORT guardedSetDebug(const std::string& s, const bool v);

  // function to check if compiled version has debug information
  bool GTSAM_EXPORT isDebugVersion();
}

#undef ISDEBUG
#undef SETDEBUG

#ifdef GTSAM_ENABLE_DEBUG

#define ISDEBUG(S) (gtsam::guardedIsDebug(S))
#define SETDEBUG(S,V) ((void)(gtsam::guardedSetDebug(S,V)))

#else

#define ISDEBUG(S) (false)
#define SETDEBUG(S,V) ((void)false)

#endif

