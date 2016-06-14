/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * LinearInequalityFactorGraph.h
 * @brief: Factor graph of all LinearInequality factors
 * @date: Dec 8, 2014
 * @author: thduynguyen
 */

#pragma once

#include <gtsam/inference/FactorGraph-inst.h>
#include <gtsam_unstable/linear/LinearInequality.h>

namespace gtsam {

class LinearInequalityFactorGraph: public FactorGraph<LinearInequality> {
private:
  typedef FactorGraph<LinearInequality> Base;

public:
  typedef boost::shared_ptr<LinearInequalityFactorGraph> shared_ptr;

  /** print */
  void print(const std::string& str, const KeyFormatter& keyFormatter =
      DefaultKeyFormatter) const {
    Base::print(str, keyFormatter);
  }

  /** equals */
  bool equals(const LinearInequalityFactorGraph& other,
      double tol = 1e-9) const {
    return Base::equals(other, tol);
  }
};

/// traits
template<> struct traits<LinearInequalityFactorGraph> : public Testable<
    LinearInequalityFactorGraph> {
};

} // \ namespace gtsam

