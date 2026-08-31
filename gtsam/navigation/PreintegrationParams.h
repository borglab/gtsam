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

#include <gtsam/base/MatrixConstants.h>
#include <gtsam/navigation/PreintegratedRotation.h>

#if GTSAM_ENABLE_BOOST_SERIALIZATION
#include <boost/serialization/version.hpp>
#endif

namespace gtsam {

/** Error chart used by IMU factors. */
enum class ImuFactorErrorMode {
  Legacy,        ///< Historical backend-dependent error chart.
  ComponentWise, ///< Use the component-wise NavState error for every backend.
  Logmap,        ///< Use the SE_2(3) NavState Logmap for every backend.
};

/// Parameters for pre-integration:
/// Usage: Create just a single Params and pass a shared pointer to the constructor
struct GTSAM_EXPORT PreintegrationParams: PreintegratedRotationParams {
  /// Continuous-time "Covariance" of accelerometer
  /// The units for stddev are σ = m/s²/√Hz
  Matrix3 accelerometerCovariance;
  Matrix3 integrationCovariance; ///< continuous-time "Covariance" describing integration uncertainty

 protected:
  friend class PreintegrationBase;

  /// Retained only for source and archive compatibility. Exact rotating-Earth
  /// dynamics are used whenever omegaCoriolis is set, regardless of this flag.
  bool use2ndOrderCoriolis;

 public:
  Vector3 n_gravity; ///< Gravity vector in nav frame

 private:
  ImuFactorErrorMode imuFactorErrorMode_;

 public:
  /// Default constructor for serialization only; fresh params use Logmap.
  PreintegrationParams()
      : PreintegratedRotationParams(),
        accelerometerCovariance(I_3x3),
        integrationCovariance(I_3x3),
        use2ndOrderCoriolis(false),
        n_gravity(0, 0, -1),
        imuFactorErrorMode_(ImuFactorErrorMode::Logmap) {}

  /// The Params constructor insists on getting the navigation frame gravity vector
  /// For convenience, two commonly used conventions are provided by named constructors below
  PreintegrationParams(const Vector3& n_gravity_)
      : PreintegratedRotationParams(),
        accelerometerCovariance(I_3x3),
        integrationCovariance(I_3x3),
        use2ndOrderCoriolis(false),
        n_gravity(n_gravity_),
        imuFactorErrorMode_(ImuFactorErrorMode::Logmap) {}

  // Default Params for a Z-down navigation frame, such as NED: gravity points along positive Z-axis
  static std::shared_ptr<PreintegrationParams> MakeSharedD(double g = 9.81) {
    return std::shared_ptr<PreintegrationParams>(new PreintegrationParams(Vector3(0, 0, g)));
  }

  // Default Params for a Z-up navigation frame, such as ENU: gravity points along negative Z-axis
  static std::shared_ptr<PreintegrationParams> MakeSharedU(double g = 9.81) {
    return std::shared_ptr<PreintegrationParams>(new PreintegrationParams(Vector3(0, 0, -g)));
  }

  void print(const std::string& s="") const override;
  bool equals(const PreintegratedRotationParams& other, double tol) const override;

  void setAccelerometerCovariance(const Matrix3& cov) { accelerometerCovariance = cov; }
  void setIntegrationCovariance(const Matrix3& cov)   { integrationCovariance = cov; }

  const Matrix3& getAccelerometerCovariance() const { return accelerometerCovariance; }
  const Matrix3& getIntegrationCovariance()   const { return integrationCovariance; }
  const Vector3& getGravity()   const { return n_gravity; }

  /**
   * Select the nonlinear error chart used by IMU factors.
   *
   * Fresh parameters default to Logmap. Legacy restores the historical
   * backend-dependent choice: component-wise for Manifold/Tangent and Logmap
   * for Lie-group/Galilean preintegration.
   */
  void setImuFactorErrorMode(ImuFactorErrorMode mode) {
    imuFactorErrorMode_ = mode;
  }

  /// Return the nonlinear error chart used by IMU factors.
  ImuFactorErrorMode getImuFactorErrorMode() const {
    return imuFactorErrorMode_;
  }

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V43
  /// @deprecated Exact rotating-Earth dynamics ignore this compatibility flag.
  void setUse2ndOrderCoriolis(bool flag) {
    use2ndOrderCoriolis = flag;
  }

  /// @deprecated Exact rotating-Earth dynamics ignore this compatibility flag.
  bool getUse2ndOrderCoriolis() const {
    return use2ndOrderCoriolis;
  }
#endif

protected:

#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    namespace bs = ::boost::serialization;
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(PreintegratedRotationParams);
    ar & BOOST_SERIALIZATION_NVP(accelerometerCovariance);
    ar & BOOST_SERIALIZATION_NVP(integrationCovariance);
    ar & BOOST_SERIALIZATION_NVP(use2ndOrderCoriolis);
    ar & BOOST_SERIALIZATION_NVP(n_gravity);
    if (version > 0) {
      ar & BOOST_SERIALIZATION_NVP(imuFactorErrorMode_);
    } else if (ARCHIVE::is_loading::value) {
      imuFactorErrorMode_ = ImuFactorErrorMode::Legacy;
    }
  }
#endif
};

} // namespace gtsam

#if GTSAM_ENABLE_BOOST_SERIALIZATION
BOOST_CLASS_VERSION(gtsam::PreintegrationParams, 1)
#endif
