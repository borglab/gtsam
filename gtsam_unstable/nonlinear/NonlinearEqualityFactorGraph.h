/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    NonlinearEqualityFactorGraph.h
 * @author  Duy-Nguyen Ta
 * @author  Krunal Chande
 * @author  Luca Carlone
 * @date    Dec 15, 2014
 */

#pragma once
#include <gtsam_unstable/nonlinear/LinearEqualityManifoldFactorGraph.h>


namespace gtsam {
class NonlinearEqualityFactorGraph: public LinearEqualityManifoldFactorGraph {
public:
  /// default constructor
  NonlinearEqualityFactorGraph() {
  }

  GaussianFactorGraph::shared_ptr multipliedHessians(const Values& values, const VectorValues& duals) const {
    GaussianFactorGraph::shared_ptr constrainedHessians(new GaussianFactorGraph());
    BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, *this) {
      NonlinearConstraint::shared_ptr constraint = boost::dynamic_pointer_cast<NonlinearConstraint>(factor);
      constrainedHessians->push_back(constraint->multipliedHessian(values, duals));
    }
    return constrainedHessians;
  }
};
}
