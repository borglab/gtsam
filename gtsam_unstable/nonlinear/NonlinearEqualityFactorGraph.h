/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    NonlinearEqualityFactorGraph.h
 * @author Ivan Dario Jimenez
 * @author  Duy-Nguyen Ta
 * @author  Krunal Chande
 * @author  Luca Carlone
 * @date    Dec 15, 2014
 */

#pragma once
#include <gtsam_unstable/linear/EqualityFactorGraph.h>
#include <gtsam_unstable/nonlinear/NonlinearEqualityConstraint.h>

namespace gtsam {

class NonlinearEqualityFactorGraph: public FactorGraph<NonlinearConstraint> {
public:
  typedef boost::shared_ptr<NonlinearEqualityFactorGraph> shared_ptr;
  /// Default constructor
  NonlinearEqualityFactorGraph() {
  }

  /// Linearize to a EqualityFactorGraph
  EqualityFactorGraph::shared_ptr linearize(
      const Values& linearizationPoint) const {
    EqualityFactorGraph::shared_ptr linearGraph(new EqualityFactorGraph());
    for (const NonlinearFactor::shared_ptr& factor : *this) {
      JacobianFactor::shared_ptr jacobian = boost::dynamic_pointer_cast
          < JacobianFactor > (factor->linearize(linearizationPoint));
      NonlinearConstraint::shared_ptr constraint = boost::dynamic_pointer_cast
          < NonlinearConstraint > (factor);
      linearGraph->add(LinearEquality(*jacobian, constraint->dualKey()));
    }
    return linearGraph;
  }

  double error(const Values& values) const {
    double total_cost(0.0);
    for (const sharedFactor& factor : *this) {
      if (factor)
        total_cost += std::abs(factor->unwhitenedError(values).sum());
    }
    return total_cost;
  }
 /**
  * Additional cost for -lambda*ConstraintHessian for SQP
  */
  GaussianFactorGraph::shared_ptr multipliedHessians(const Values & values, const VectorValues & duals) const{
    GaussianFactorGraph::shared_ptr constrainedHessians(new GaussianFactorGraph());
    for(const NonlinearConstraint::shared_ptr& factor: *this){
      constrainedHessians->push_back(factor->multipliedHessian(values, duals));
    }
    return constrainedHessians;
  }
};

}
