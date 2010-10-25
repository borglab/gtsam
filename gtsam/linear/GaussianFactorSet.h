/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussianFactorSet.h
 * @brief   Utility class: an STL set of linear factors, basically a wrappable typedef
 * @author  Frank Dellaert
 */

#pragma once

#include <set>
#include <boost/shared_ptr.hpp>
#include <gtsam/linear/GaussianFactor.h>

namespace gtsam {
  
  class GaussianFactor;
  
  // We use a vector not a an STL set, to get predictable ordering across platforms
  struct GaussianFactorSet : std::vector<boost::shared_ptr<GaussianFactor> > {
    GaussianFactorSet() {}
  };
}
