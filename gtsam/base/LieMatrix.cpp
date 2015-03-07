/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file LieMatrix.cpp
 * @brief A wrapper around Matrix providing Lie compatibility
 * @author Richard Roberts and Alex Cunningham
 */

#include <gtsam/base/LieMatrix.h>

namespace gtsam {

/* ************************************************************************* */
void LieMatrix::print(const std::string& name) const {
  gtsam::print(matrix(), name);
}

}
