/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    debug.cpp
 * @brief   Global debugging flags
 * @author  Richard Roberts
 * @date    Feb 1, 2011
 */

#include <gtsam/base/debug.h>
#include <gtsam/config.h> // for GTSAM_USE_TBB

#ifdef GTSAM_USE_TBB
#include <mutex> // std::mutex, std::unique_lock
#endif

namespace gtsam {

GTSAM_EXPORT FastMap<std::string, ValueWithDefault<bool, false> > debugFlags;

#ifdef GTSAM_USE_TBB
std::mutex debugFlagsMutex;
#endif

/* ************************************************************************* */
bool guardedIsDebug(const std::string& s) {
#ifdef GTSAM_USE_TBB
  std::unique_lock<std::mutex> lock(debugFlagsMutex);
#endif
  return gtsam::debugFlags[s];
}

/* ************************************************************************* */
void guardedSetDebug(const std::string& s, const bool v) {
#ifdef GTSAM_USE_TBB
  std::unique_lock<std::mutex> lock(debugFlagsMutex);
#endif
  gtsam::debugFlags[s] = v;
}

bool isDebugVersion() {
#ifdef NDEBUG
  // nondebug
  return false;
#else
  // debug
  return true;
#endif

}

}
