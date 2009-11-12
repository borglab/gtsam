/**
 * @file    GaussianFactorSet.h
 * @brief   Utility class: an STL set of linear factors, basically a wrappable typedef
 * @author  Frank Dellaert
 */

#pragma once

#include <set>
#include <boost/shared_ptr.hpp>
#include "GaussianFactor.h"

namespace gtsam {
  
  class GaussianFactor;
  
  // We use a vector not a an STL set, to get predictable ordering across platforms
  struct GaussianFactorSet : std::vector<boost::shared_ptr<GaussianFactor> > {
    GaussianFactorSet() {}
  };
}
