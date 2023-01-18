/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   TangentPreintegration.h
 *  @author Frank Dellaert
 *  @author Adam Bry
 **/

#pragma once

#include <gtsam/navigation/PreintegrationBase.h>

namespace gtsam {

/**
 * Integrate on the 9D tangent space of the NavState manifold.
 * See extensive discussion in ImuFactor.lyx
 */
class GTSAM_EXPORT TangentPreintegration : public PreintegrationBase {
 protected:

  /**
   * Preintegrated navigation state, as a 9D vector on tangent space at frame i
   * Order is: theta, position, velocity
   */
  Vector9 preintegrated_;
  Matrix93 preintegrated_H_biasAcc_;    ///< Jacobian of preintegrated_ w.r.t. acceleration bias
  Matrix93 preintegrated_H_biasOmega_;  ///< Jacobian of preintegrated_ w.r.t. angular rate bias

  /// Default constructor for serialization
  TangentPreintegration() {
    resetIntegration();
  }

public:
  /// @name Constructors/destructors
  /// @{

  /**
   *  Constructor, initializes the variables in the base class
   *  @param p    Parameters, typically fixed in a single application
   *  @param bias Current estimate of acceleration and rotation rate biases
   */
  TangentPreintegration(const std::shared_ptr<Params>& p,
      const imuBias::ConstantBias& biasHat = imuBias::ConstantBias());

  /// Virtual destructor
  ~TangentPreintegration() override {
  }

  /// @}

  /// @name Basic utilities
  /// @{
  /// Re-initialize PreintegratedMeasurements
  void resetIntegration() override;

  /// @}

  /// @name Instance variables access
  /// @{
  Vector3  deltaPij() const override { return preintegrated_.segment<3>(3); }
  Vector3  deltaVij() const override { return preintegrated_.tail<3>(); }
  Rot3     deltaRij() const override { return Rot3::Expmap(theta()); }
  NavState deltaXij() const override { return NavState().retract(preintegrated_); }

  const Vector9& preintegrated() const { return preintegrated_; }
  Vector3 theta() const     { return preintegrated_.head<3>(); }
  const Matrix93& preintegrated_H_biasAcc() const { return preintegrated_H_biasAcc_; }
  const Matrix93& preintegrated_H_biasOmega() const { return preintegrated_H_biasOmega_; }

  /// @name Testable
  /// @{
  bool equals(const TangentPreintegration& other, double tol) const;
  /// @}

  /// @name Main functionality
  /// @{

  // Update integrated vector on tangent manifold preintegrated with acceleration
  // Static, functional version.
  static Vector9 UpdatePreintegrated(const Vector3& a_body,
                                     const Vector3& w_body, const double dt,
                                     const Vector9& preintegrated,
                                     OptionalJacobian<9, 9> A = {},
                                     OptionalJacobian<9, 3> B = {},
                                     OptionalJacobian<9, 3> C = {});

  /// Update preintegrated measurements and get derivatives
  /// It takes measured quantities in the j frame
  /// Modifies preintegrated quantities in place after correcting for bias and possibly sensor pose
  /// NOTE(frank): implementation is different in two versions
  void update(const Vector3& measuredAcc, const Vector3& measuredOmega,
      const double dt, Matrix9* A, Matrix93* B, Matrix93* C) override;

  /// Given the estimate of the bias, return a NavState tangent vector
  /// summarizing the preintegrated IMU measurements so far
  /// NOTE(frank): implementation is different in two versions
  Vector9 biasCorrectedDelta(const imuBias::ConstantBias& bias_i,
      OptionalJacobian<9, 6> H = {}) const override;

  // Compose the two pre-integrated 9D-vectors zeta01 and zeta02, with derivatives
  static Vector9 Compose(const Vector9& zeta01, const Vector9& zeta12,
                         double deltaT12,
                         OptionalJacobian<9, 9> H1 = {},
                         OptionalJacobian<9, 9> H2 = {});

  /// Merge in a different set of measurements and update bias derivatives accordingly
  /// The derivatives apply to the preintegrated Vector9
  void mergeWith(const TangentPreintegration& pim, Matrix9* H1, Matrix9* H2);
  /// @}

  /** Dummy clone for MATLAB */
  virtual std::shared_ptr<TangentPreintegration> clone() const {
    return std::shared_ptr<TangentPreintegration>();
  }

  /// @}

private:
  /** Serialization function */
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    namespace bs = ::boost::serialization;
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(PreintegrationBase);
    ar & BOOST_SERIALIZATION_NVP(preintegrated_);
    ar & BOOST_SERIALIZATION_NVP(preintegrated_H_biasAcc_);
    ar & BOOST_SERIALIZATION_NVP(preintegrated_H_biasOmega_);
  }
#endif

public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW
};

} /// namespace gtsam
