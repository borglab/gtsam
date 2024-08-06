/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    NonlinearConstraint.h
 * @brief   Nonlinear constraints in constrained optimization.
 * @author  Yetong Zhang, Frank Dellaert
 * @date    Aug 3, 2024
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
#include <boost/serialization/base_object.hpp>
#endif

namespace gtsam {

/**
 * Base class for nonlinear constraint.
 * The constraint is represented as a NoiseModelFactor with Constrained noise model.
 * whitenedError() returns The constraint violation vector.
 * unwhitenedError() returns the sigma-scaled constraint violation vector.
 */
class NonlinearConstraint : public NoiseModelFactor {
 public:
  typedef NoiseModelFactor Base;

  /** Use constructors of NoiseModelFactor. */
  using Base::Base;

  /** Destructor. */
  virtual ~NonlinearConstraint() {}

  /** Create a cost factor representing the L2 penalty function with scaling coefficient mu. */
  virtual NoiseModelFactor::shared_ptr penaltyFactor(const double mu = 1.0) const {
    return cloneWithNewNoiseModel(penaltyNoise(mu));
  }

  /** Return the norm of the constraint violation vector. */
  virtual double violation(const Values& x) const { return sqrt(2 * error(x)); }

  /**  Check if constraint violation is within tolerance. */
  virtual bool feasible(const Values& x, const double tolerance = 1e-5) const {
    return violation(x) <= tolerance;
  }

 protected:
  /** Noise model used for the penalty function. */
  SharedDiagonal penaltyNoise(const double mu) const {
    return noiseModel::Diagonal::Sigmas(noiseModel()->sigmas() / sqrt(mu));
  }

  /** Default constrained noisemodel used for construction of constraint. */
  static SharedNoiseModel constrainedNoise(const Vector& sigmas) {
    return noiseModel::Constrained::MixedSigmas(1.0, sigmas);
  }

 private:
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp("NonlinearConstraint",
                                       boost::serialization::base_object<Base>(*this));
  }
#endif
};

}  // namespace gtsam
