/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    NonlinearEqualityFactorGraph.h
 * @author  Ivan Dario Jimenez
 * @date    Dec 15, 2014
 */

#include <gtsam_unstable/linear/EqualityFactorGraph.h>
#include <gtsam_unstable/nonlinear/NonlinearEqualityConstraint.h>

namespace gtsam {
class NonlinearCostFactorGraph: public FactorGraph<NonlinearConstraint> {
public:

  NonlinearCostFactorGraph() {
  }

  GaussianFactorGraph::shared_ptr linearize(const Values& linearizationPoint) const {
    GaussianFactorGraph::shared_ptr linearizedGraph(new GaussianFactorGraph());

    for (const NonlinearFactor::shared_ptr & factor : *this) {
      JacobianFactor::shared_ptr jacobian = boost::dynamic_pointer_cast
        < JacobianFactor > (factor->linearize(linearizationPoint));
      linearizedGraph->push_back(*jacobian);
    }
    return linearizedGraph;
  }

  double error(const Values & values) const {
    double total_error(0.0);

    for (const sharedFactor & factor : *this) {
      if (factor)
        total_error += std::abs(factor->unwhitenedError(values).sum());
    }

    return total_error;
  }
};
}
