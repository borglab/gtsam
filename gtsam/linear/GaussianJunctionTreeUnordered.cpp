/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file GaussianJunctionTreeUnordered.cpp
 * @date Mar 29, 2013
 * @author Frank Dellaert
 * @author Richard Roberts
 */

#include <gtsam/inference/JunctionTreeUnordered-inst.h>
#include <gtsam/linear/GaussianJunctionTreeUnordered.h>
#include <gtsam/linear/GaussianEliminationTreeUnordered.h>

namespace gtsam {

  /* ************************************************************************* */
  GaussianJunctionTreeUnordered::GaussianJunctionTreeUnordered(
    const GaussianEliminationTreeUnordered& eliminationTree) :
  Base(Base::FromEliminationTree(eliminationTree)) {}

  /* ************************************************************************* */
  GaussianJunctionTreeUnordered::GaussianJunctionTreeUnordered(const This& other) :
    Base(other) {}

  /* ************************************************************************* */
  GaussianJunctionTreeUnordered& GaussianJunctionTreeUnordered::operator=(const This& other)
  {
    (void) Base::operator=(other);
    return *this;
  }

}
