/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  CombinedImuFactor.cpp
 *  @author Frank Dellaert
 *  @date Nov 28, 2014
 **/

#include <gtsam/navigation/CombinedImuFactor.h>

namespace gtsam {

const Matrix3 CombinedImuFactor::Z_3x3 = Matrix3::Zero();
const Matrix3 CombinedImuFactor::I_3x3 = Matrix3::Identity();

} /// namespace gtsam
