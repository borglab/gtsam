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

namespace gtsam {

  FastMap<std::string, ValueWithDefault<bool,false> > debugFlags;

}
