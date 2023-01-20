/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  PreintegrationParams.h
 *  @author Frank Dellaert
 **/

#pragma once

#include <gtsam/navigation/PreintegratedRotation.h>
#include <boost/make_shared.hpp>

namespace gtsam {

/// Parameters for pre-integration:
/// Usage: Create just a single Params and pass a shared pointer to the constructor
struct GTSAM_EXPORT PreintegrationParams: PreintegratedRotationParams {
  /// Continuous-time "Covariance" of accelerometer
  /// The units for stddev are σ = m/s²/√Hz
  Matrix3 accelerometerCovariance;
  Matrix3 integrationCovariance; ///< continuous-time "Covariance" describing integration uncertainty
  bool use2ndOrderCoriolis; ///< Whether to use second order Coriolis integration
  Vector3 n_gravity; ///< Gravity vector in nav frame

  /// Default constructor for serialization only
  PreintegrationParams()
      : PreintegratedRotationParams(),
        accelerometerCovariance(I_3x3),
        integrationCovariance(I_3x3),
        use2ndOrderCoriolis(false),
        n_gravity(0, 0, -1) {}

  /// The Params constructor insists on getting the navigation frame gravity vector
  /// For convenience, two commonly used conventions are provided by named constructors below
  PreintegrationParams(const Vector3& n_gravity)
      : PreintegratedRotationParams(),
        accelerometerCovariance(I_3x3),
        integrationCovariance(I_3x3),
        use2ndOrderCoriolis(false),
        n_gravity(n_gravity) {}

  // Default Params for a Z-down navigation frame, such as NED: gravity points along positive Z-axis
  static boost::shared_ptr<PreintegrationParams> MakeSharedD(double g = 9.81) {
    return boost::shared_ptr<PreintegrationParams>(new PreintegrationParams(Vector3(0, 0, g)));
  }

  // Default Params for a Z-up navigation frame, such as ENU: gravity points along negative Z-axis
  static boost::shared_ptr<PreintegrationParams> MakeSharedU(double g = 9.81) {
    return boost::shared_ptr<PreintegrationParams>(new PreintegrationParams(Vector3(0, 0, -g)));
  }

  void print(const std::string& s="") const override;
  bool equals(const PreintegratedRotationParams& other, double tol) const override;

  void setAccelerometerCovariance(const Matrix3& cov) { accelerometerCovariance = cov; }
  void setIntegrationCovariance(const Matrix3& cov)   { integrationCovariance = cov; }
  void setUse2ndOrderCoriolis(bool flag)              { use2ndOrderCoriolis = flag; }

  const Matrix3& getAccelerometerCovariance() const { return accelerometerCovariance; }
  const Matrix3& getIntegrationCovariance()   const { return integrationCovariance; }
  const Vector3& getGravity()   const { return n_gravity; }
  bool           getUse2ndOrderCoriolis()     const { return use2ndOrderCoriolis; }

protected:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    namespace bs = ::boost::serialization;
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(PreintegratedRotationParams);
    ar & BOOST_SERIALIZATION_NVP(accelerometerCovariance);
    ar & BOOST_SERIALIZATION_NVP(integrationCovariance);
    ar & BOOST_SERIALIZATION_NVP(use2ndOrderCoriolis);
    ar & BOOST_SERIALIZATION_NVP(n_gravity);
  }

#ifdef GTSAM_USE_QUATERNIONS
  // Align if we are using Quaternions
public:
	GTSAM_MAKE_ALIGNED_OPERATOR_NEW
#endif
};

} // namespace gtsam
