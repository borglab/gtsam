/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    InequalityFactorGraph.h
 * @brief   Factor graph of all LinearInequality factors
 * @date    Dec 8, 2014
 * @author  Duy-Nguyen Ta
 */

#pragma once

#include <gtsam_unstable/linear/LinearInequality.h>
#include <gtsam/inference/FactorGraph-inst.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/inference/FactorGraph.h>

namespace gtsam {

/**
 * Collection of all Linear Inequality constraints Ax-b <= 0 of
 * a Programming problem as a Factor Graph
 */
class InequalityFactorGraph: public FactorGraph<LinearInequality> {
private:
  typedef FactorGraph<LinearInequality> Base;

public:
  typedef std::shared_ptr<InequalityFactorGraph> shared_ptr;

  /** print */
  void print(
      const std::string& str = "",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    Base::print(str, keyFormatter);
  }

  /** equals */
  bool equals(const InequalityFactorGraph& other, double tol = 1e-9) const {
    return Base::equals(other, tol);
  }

  /// Add a linear inequality, forwards arguments to LinearInequality.
  template <class... Args> void add(Args &&... args) {
    emplace_shared<LinearInequality>(std::forward<Args>(args)...);
  }

  /**
   * Compute error of a guess.
   * Infinity error if it violates an inequality; zero otherwise. */
  double error(const VectorValues& x) const {
    for (const sharedFactor& factor : *this) {
      if (factor)
        if (factor->error(x) > 1e-7)
          return std::numeric_limits<double>::infinity();
    }
    return 0.0;
  }
};

/// traits
template<>
struct traits<InequalityFactorGraph> : public Testable<InequalityFactorGraph> {
};

} // \ namespace gtsam

