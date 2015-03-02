/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    LinearEqualityFactorGraph.h
 * @author  Duy-Nguyen Ta
 * @author  Krunal Chande
 * @author  Luca Carlone
 * @date    Dec 15, 2014
 */

#pragma once
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam_unstable/linear/EqualityFactorGraph.h>

namespace gtsam {

/**
 * FactorGraph container for linear equality factors c(x)==0 at the nonlinear level
 */
class LinearEqualityFactorGraph: public FactorGraph<NonlinearFactor> {
public:
  /// Default constructor
  LinearEqualityFactorGraph() {
  }

  /// Linearize to a EqualityFactorGraph
  EqualityFactorGraph::shared_ptr linearize(const Values& linearizationPoint) const;

  /// Return true if the max absolute error all factors is less than a tolerance
  bool checkFeasibility(const Values& values, double tol) const;

};

}
