/**
 * @file    LinearFactorSet.h
 * @brief   Utility class: an STL set of linear factors, basically a wrappable typedef
 * @author  Frank Dellaert
 */

#pragma once

#include <set>
#include <boost/shared_ptr.hpp>
#include "LinearFactor.h"

namespace gtsam {
  
  class LinearFactor;
  
  // We use a vector not a an STL set, to get predictable ordering across platforms
  struct LinearFactorSet : std::vector<boost::shared_ptr<LinearFactor> > {
    LinearFactorSet() {}
  };
}
