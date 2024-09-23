/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  PreintegrationCombinedParams.h
 *  @author Luca Carlone
 *  @author Stephen Williams
 *  @author Richard Roberts
 *  @author Vadim Indelman
 *  @author David Jensen
 *  @author Frank Dellaert
 *  @author Varun Agrawal
 **/

#pragma once

/* GTSAM includes */
#include <gtsam/base/Matrix.h>
#include <gtsam/navigation/ManifoldPreintegration.h>
#include <gtsam/navigation/TangentPreintegration.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

/// Parameters for pre-integration using PreintegratedCombinedMeasurements:
/// Usage: Create just a single Params and pass a shared pointer to the
/// constructor
struct GTSAM_EXPORT PreintegrationCombinedParams : PreintegrationParams {
  Matrix3 biasAccCovariance;    ///< continuous-time "Covariance" describing
                                ///< accelerometer bias random walk
  Matrix3 biasOmegaCovariance;  ///< continuous-time "Covariance" describing
                                ///< gyroscope bias random walk
  Matrix6 biasAccOmegaInt;  ///< covariance of bias used as initial estimate.

  /// Default constructor makes uninitialized params struct.
  /// Used for serialization.
  PreintegrationCombinedParams()
      : biasAccCovariance(I_3x3),
        biasOmegaCovariance(I_3x3),
        biasAccOmegaInt(I_6x6) {}

  /// See two named constructors below for good values of n_gravity in body
  /// frame
  PreintegrationCombinedParams(const Vector3& n_gravity_)
      : PreintegrationParams(n_gravity_),
        biasAccCovariance(I_3x3),
        biasOmegaCovariance(I_3x3),
        biasAccOmegaInt(I_6x6) {}

  // Default Params for a Z-down navigation frame, such as NED: gravity points
  // along positive Z-axis
  static std::shared_ptr<PreintegrationCombinedParams> MakeSharedD(
      double g = 9.81) {
    return std::shared_ptr<PreintegrationCombinedParams>(
        new PreintegrationCombinedParams(Vector3(0, 0, g)));
  }

  // Default Params for a Z-up navigation frame, such as ENU: gravity points
  // along negative Z-axis
  static std::shared_ptr<PreintegrationCombinedParams> MakeSharedU(
      double g = 9.81) {
    return std::shared_ptr<PreintegrationCombinedParams>(
        new PreintegrationCombinedParams(Vector3(0, 0, -g)));
  }

  void print(const std::string& s = "") const override;
  bool equals(const PreintegratedRotationParams& other,
              double tol) const override;

  void setBiasAccCovariance(const Matrix3& cov) { biasAccCovariance = cov; }
  void setBiasOmegaCovariance(const Matrix3& cov) { biasOmegaCovariance = cov; }
  void setBiasAccOmegaInit(const Matrix6& cov) { biasAccOmegaInt = cov; }

  const Matrix3& getBiasAccCovariance() const { return biasAccCovariance; }
  const Matrix3& getBiasOmegaCovariance() const { return biasOmegaCovariance; }
  const Matrix6& getBiasAccOmegaInit() const { return biasAccOmegaInt; }

 private:
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    namespace bs = ::boost::serialization;
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(PreintegrationParams);
    ar& BOOST_SERIALIZATION_NVP(biasAccCovariance);
    ar& BOOST_SERIALIZATION_NVP(biasOmegaCovariance);
    ar& BOOST_SERIALIZATION_NVP(biasAccOmegaInt);
  }
#endif

 public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace gtsam
