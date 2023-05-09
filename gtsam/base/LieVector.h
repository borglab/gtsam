/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file LieVector.h
 * @brief Deprecation warning for LieVector, see deprecated/LieVector.h for details.
 * @author Paul Drews
 */

#pragma once

#ifdef _MSC_VER
#pragma message("LieVector.h is deprecated. Please use Eigen::Vector instead.")
#else
#warning "LieVector.h is deprecated. Please use Eigen::Vector instead."
#endif

#include <gtsam/base/deprecated/LieVector.h>
