/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    EqualityFactorGraph.h
 * @brief   Factor graph of all LinearEquality factors
 * @date    Dec 8, 2014
 * @author  Duy-Nguyen Ta
 */

#pragma once

#include <gtsam/inference/FactorGraph.h>
#include <gtsam_unstable/linear/LinearEquality.h>

namespace gtsam {
/**
 * This class is used to represent an equality constraint on
 * a Programming problem of the form Ax = b.
 */
class EqualityFactorGraph : public FactorGraph<LinearEquality> {
public:
  typedef boost::shared_ptr<EqualityFactorGraph> shared_ptr;

  /** compute error of a guess.
   * TODO: This code is duplicated in GaussianFactorGraph and NonlinearFactorGraph!!
   * Remove it!
   */
  double error(const VectorValues& x) const {
    double total_error = 0.;
    for(const sharedFactor& factor: *this){
      if(factor)
        total_error += factor->error(x);
    }
    return total_error;
  }
};

/// traits
template<> struct traits<EqualityFactorGraph> : public Testable<
    EqualityFactorGraph> {
};

} // \ namespace gtsam

