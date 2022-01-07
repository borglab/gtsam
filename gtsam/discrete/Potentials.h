/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Potentials.h
 * @date March 24, 2011
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/discrete/AlgebraicDecisionTree.h>
#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam/inference/Key.h>

#include <boost/shared_ptr.hpp>
#include <set>

namespace gtsam {

  /*
   * @deprecated
   * @brief Deprecated class for storing an ADT with some convenience methods
   * */
  typedef GTSAM_DEPRECATED AlgebraicDecisionTree<Key> Potentials;

} // namespace gtsam
