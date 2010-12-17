/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timing.cpp
 * @brief   
 * @author  Richard Roberts (extracted from Michael Kaess' timing functions)
 * @created Oct 5, 2010
 */

#include <gtsam/base/timing.h>

boost::shared_ptr<TimingOutline> timingRoot(new TimingOutline("Total"));
boost::weak_ptr<TimingOutline> timingCurrent(timingRoot);

Timing timing;
std::string timingPrefix;

