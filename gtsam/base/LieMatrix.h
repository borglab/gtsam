/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file LieMatrix.h
 * @brief External deprecation warning, see deprecated/LieMatrix.h for details
 * @author Paul Drews
 */

#pragma once

#ifdef _MSC_VER
#pragma message("LieMatrix.h is deprecated. Please use Eigen::Matrix instead.")
#else
#warning "LieMatrix.h is deprecated. Please use Eigen::Matrix instead."
#endif

#include "gtsam/base/deprecated/LieMatrix.h"
