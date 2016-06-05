/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  ManifoldPreintegration.h
 *  @author Luca Carlone
 *  @author Stephen Williams
 *  @author Richard Roberts
 *  @author Vadim Indelman
 *  @author David Jensen
 *  @author Frank Dellaert
 **/

#pragma once

#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/PreintegrationBase.h>

namespace gtsam {

/**
 * IMU pre-integration on NavSatet manifold.
 * This corresponds to the original RSS paper (with one difference: V is rotated)
 */
class ManifoldPreintegration : public PreintegrationBase {
 protected:

  /**
   * Pre-integrated navigation state, from frame i to frame j
   * Note: relative position does not take into account velocity at time i, see deltap+, in [2]
   * Note: velocity is now also in frame i, as opposed to deltaVij in [2]
   */
  NavState deltaXij_;
  // TODO(Luca): Move to base
  Matrix93 preintegrated_H_biasAcc_;    ///< Jacobian of deltaXij_ w.r.t. acceleration bias
  Matrix93 preintegrated_H_biasOmega_;  ///< Jacobian of deltaXij_ w.r.t. angular rate bias

  /// Default constructor for serialization
  ManifoldPreintegration() {
    resetIntegration();
  }

public:
  /// @name Constructors
  /// @{

  /**
   *  Constructor, initializes the variables in the base class
   *  @param p    Parameters, typically fixed in a single application
   *  @param bias Current estimate of acceleration and rotation rate biases
   */
  ManifoldPreintegration(const boost::shared_ptr<Params>& p,
      const imuBias::ConstantBias& biasHat = imuBias::ConstantBias());

  /// @}

  /// @name Basic utilities
  /// @{
  /// Re-initialize PreintegratedMeasurements
  void resetIntegration() override;

  /// @}

  /// @name Instance variables access
  /// @{
  NavState deltaXij() const override { return deltaXij_; }
  Rot3     deltaRij() const override { return deltaXij_.attitude(); }
  Vector3  deltaPij() const override { return deltaXij_.position().vector(); }
  Vector3  deltaVij() const override { return deltaXij_.velocity(); }

  const Matrix93 preintegrated_H_biasAcc() const {
    return preintegrated_H_biasAcc_;
  }
  const Matrix93 preintegrated_H_biasOmega() const {
    return preintegrated_H_biasOmega_;
  }

  /// @name Testable
  /// @{
  bool equals(const ManifoldPreintegration& other, double tol) const;
  /// @}

  /// @name Main functionality
  /// @{

  /// Update preintegrated measurements and get derivatives
  /// It takes measured quantities in the j frame
  /// Modifies preintegrated quantities in place after correcting for bias and possibly sensor pose
  /// NOTE(frank): implementation is different in two versions
  void update(const Vector3& measuredAcc, const Vector3& measuredOmega, const double dt,
              Matrix9* A, Matrix93* B, Matrix93* C) override;

  /// Given the estimate of the bias, return a NavState tangent vector
  /// summarizing the preintegrated IMU measurements so far
  /// NOTE(frank): implementation is different in two versions
  Vector9 biasCorrectedDelta(const imuBias::ConstantBias& bias_i,
      OptionalJacobian<9, 6> H = boost::none) const override;

  /** Dummy clone for MATLAB */
  virtual boost::shared_ptr<ManifoldPreintegration> clone() const {
    return boost::shared_ptr<ManifoldPreintegration>();
  }

  /// @}

private:
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    namespace bs = ::boost::serialization;
    ar & BOOST_SERIALIZATION_NVP(p_);
    ar & BOOST_SERIALIZATION_NVP(deltaTij_);
    ar & BOOST_SERIALIZATION_NVP(deltaXij_);
    ar & BOOST_SERIALIZATION_NVP(biasHat_);
    // TODO(Luca): 2 Jacobians, in base !
  }
};

} /// namespace gtsam
