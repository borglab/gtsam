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
  
  struct LinearFactorSet : std::set<boost::shared_ptr<LinearFactor> > {
    LinearFactorSet() {}
  };
}
