/**
 * @file    debug.cpp
 * @brief   
 * @author  Richard Roberts
 * @created Feb 1, 2011
 */

#include <gtsam/base/debug.h>

namespace gtsam {

#ifdef GTSAM_ENABLE_DEBUG
  FastMap<std::string, ValueWithDefault<bool,false> > debugFlags;
#endif

}
