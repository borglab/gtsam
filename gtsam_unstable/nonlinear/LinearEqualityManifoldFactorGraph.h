/**
 * @file	LinearEqualityManifoldFactorGraph.h
 * @author  Duy-Nguyen Ta
 * @author 	Krunal Chande
 * @author  Luca Carlone
 * @date    Dec 15, 2014
 */

#pragma once
#include <gtsam/nonlinear/NonlinearFactorGraph.h>


namespace gtsam {

class LinearEqualityManifoldFactorGraph: public FactorGraph<NonlinearFactor> {
public:
  /// default constructor
  LinearEqualityManifoldFactorGraph() {
  }

  /// linearize to a LinearEqualityFactorGraph
  LinearEqualityFactorGraph::shared_ptr linearize(
      const Values& linearizationPoint) const {
    LinearEqualityFactorGraph::shared_ptr linearGraph(
        new LinearEqualityFactorGraph());
    BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, *this){
      JacobianFactor::shared_ptr jacobian = boost::dynamic_pointer_cast<JacobianFactor>(
          factor->linearize(linearizationPoint));
      NonlinearConstraint::shared_ptr constraint = boost::dynamic_pointer_cast<NonlinearConstraint>(factor);
      linearGraph->add(LinearEquality(*jacobian, constraint->dualKey()));
    }
    return linearGraph;
  }

  /**
   * Return true if the max absolute error all factors is less than a tolerance
   */
  bool checkFeasibility(const Values& values, double tol) const {
    BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, *this){
      NoiseModelFactor::shared_ptr noiseModelFactor = boost::dynamic_pointer_cast<NoiseModelFactor>(
          factor);
      Vector error = noiseModelFactor->unwhitenedError(values);
      if (error.lpNorm<Eigen::Infinity>() > tol) {
        return false;
      }
    }
    return true;
  }
};
}
