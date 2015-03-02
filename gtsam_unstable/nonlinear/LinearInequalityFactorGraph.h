/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    LinearInequalityFactorGraph.h
 * @author  Duy-Nguyen Ta
 * @author  Krunal Chande
 * @author  Luca Carlone
 * @date    Dec 15, 2014
 */

#pragma once
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam_unstable/linear/InequalityFactorGraph.h>

namespace gtsam {

/**
 * FactorGraph container for linear inequality factors c(x)<=0 at the nonlinear level
 */
class LinearInequalityFactorGraph: public FactorGraph<NonlinearFactor> {

public:
  /// Default constructor
  LinearInequalityFactorGraph() {}

  /// Linearize to a InequalityFactorGraph
  InequalityFactorGraph::shared_ptr linearize(const Values& linearizationPoint) const;

  /// Return true if the all errors are <= 0.0
  bool checkFeasibilityAndComplimentary(const Values& values,
      const VectorValues& dualValues, double tol) const;

};

} // namespace gtsam
