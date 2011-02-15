/**
 * @file    debug.cpp
 * @brief   
 * @author  Richard Roberts
 * @created Feb 1, 2011
 */

#include <gtsam/base/debug.h>

namespace gtsam {

  FastMap<std::string, ValueWithDefault<bool,false> > debugFlags;

}
