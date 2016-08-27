/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */
/**
 * @file 	 NonlinearConstraint.h
 * @brief  
 * @author Duy-Nguyen Ta
 * @author Ivan Dario Jimenez
 * @date 	 Sep 30, 2013
 */

#pragma once

#include <gtsam/base/Manifold.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

class NonlinearConstraint: public NoiseModelFactor {

protected:
  Key dualKey_;
  bool active_;
  typedef NoiseModelFactor Base;

public:
  virtual bool isActive_() const {
    return active_; // equality constraints are always active
  }

  void setActive_(bool active) {
    active_ = active;
  }

  typedef boost::shared_ptr<NonlinearConstraint> shared_ptr;

  /// Construct with dual key
  template<typename CONTAINER>
  NonlinearConstraint(const SharedNoiseModel& noiseModel, const CONTAINER& keys,
      Key dualKey, bool isActive = true) :
      Base(noiseModel, keys), dualKey_(dualKey), active_(isActive) {
  }

  /**
   * compute the HessianFactor of the (-dual * constraintHessian) for the qp subproblem's objective function
   */
  virtual GaussianFactor::shared_ptr multipliedHessian(const Values& x,
      const VectorValues& duals) const = 0;

  /// return the dual key
  Key dualKey() const {
    return dualKey_;
  }
};

} /* namespace gtsam */
