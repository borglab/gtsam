/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    LinearEqualityFactorGraph.cpp
 * @author  Duy-Nguyen Ta
 * @author  Krunal Chande
 * @author  Luca Carlone
 * @date    Dec 15, 2014
 */

#include <gtsam_unstable/nonlinear/LinearEqualityFactorGraph.h>
#include <gtsam_unstable/nonlinear/ConstrainedFactor.h>

namespace gtsam {

/* ************************************************************************* */
EqualityFactorGraph::shared_ptr LinearEqualityFactorGraph::linearize(
    const Values& linearizationPoint) const {
  EqualityFactorGraph::shared_ptr linearGraph(
      new EqualityFactorGraph());
  BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, *this){
    JacobianFactor::shared_ptr jacobian = boost::dynamic_pointer_cast<JacobianFactor>(
        factor->linearize(linearizationPoint));
    ConstrainedFactor::shared_ptr constraint = boost::dynamic_pointer_cast<ConstrainedFactor>(factor);
    linearGraph->add(LinearEquality(*jacobian, constraint->dualKey()));
  }
  return linearGraph;
}

/* ************************************************************************* */
bool LinearEqualityFactorGraph::checkFeasibility(const Values& values, double tol) const {
  BOOST_FOREACH(const NonlinearFactor::shared_ptr& factor, *this){
    NoiseModelFactor::shared_ptr noiseModelFactor
        = boost::dynamic_pointer_cast<NoiseModelFactor>(factor);
    Vector error = noiseModelFactor->unwhitenedError(values);

    if (error.lpNorm<Eigen::Infinity>() > tol)
      return false;
  }
  return true;
}

}
