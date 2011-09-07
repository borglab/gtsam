/**
 * @file    debug.cpp
 * @brief   Global debugging flags
 * @author  Richard Roberts
 * @date    Feb 1, 2011
 */

#include <gtsam/base/debug.h>

namespace gtsam {

  FastMap<std::string, ValueWithDefault<bool,false> > debugFlags;

}
