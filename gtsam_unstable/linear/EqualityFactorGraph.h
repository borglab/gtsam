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
 * Collection of all Linear Equality constraints Ax=b of
 * a Programming problem as a Factor Graph
 */
class EqualityFactorGraph: public FactorGraph<LinearEquality> {
public:
  typedef std::shared_ptr<EqualityFactorGraph> shared_ptr;

  /// Add a linear inequality, forwards arguments to LinearInequality.
  template <class... Args> void add(Args &&... args) {
    emplace_shared<LinearEquality>(std::forward<Args>(args)...);
  }

  /// Compute error of a guess.
  double error(const VectorValues& x) const {
    double total_error = 0.;
    for (const sharedFactor& factor : *this) {
      if (factor)
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

