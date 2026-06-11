/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   AttitudeFactor.h
 *  @author Frank Dellaert
 *  @brief  Header file for Attitude factor
 *  @date   January 28, 2014
 **/
#pragma once

#include <gtsam/geometry/ExtendedPose3.h>
#include <gtsam/geometry/Gal3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>

#include <type_traits>

namespace gtsam {

namespace detail {

// Type trait to detect ExtendedPose3 instantiations
template <typename T>
struct is_extended_pose3 : std::false_type {};

template <int K, class Derived>
struct is_extended_pose3<gtsam::ExtendedPose3<K, Derived>> : std::true_type {};

template <typename T>
constexpr bool is_extended_pose3_v = is_extended_pose3<T>::value;
template <class VALUE>
inline std::string attitudeFactorName() {
  if constexpr (std::is_same_v<VALUE, Rot3>) {
    return "AttitudeFactorRot3";
  } else if constexpr (std::is_same_v<VALUE, Pose3>) {
    return "AttitudeFactorPose3";
  } else if constexpr (std::is_same_v<VALUE, NavState>) {
    return "AttitudeFactorNavState";
  } else if constexpr (std::is_same_v<VALUE, Gal3>) {
    return "AttitudeFactorGal3";
  } else if constexpr (std::is_same_v<VALUE, Se23>) {
    return "AttitudeFactorSe23";
  } else if constexpr (is_extended_pose3_v<VALUE>) {
    constexpr int K = std::remove_reference_t<VALUE>::K;
    const std::string valueName = (K == Eigen::Dynamic)
                                      ? "ExtendedPose3d"
                                      : "ExtendedPose3" + std::to_string(K);
    return "AttitudeFactor" + valueName;
  } else {
    return "AttitudeFactor";
  }
}

}  // namespace detail

/**
 * @class AttitudeFactor
 *
 * @brief Unary factor that constrains the rotation component of a value.
 *
 * The error is zero when the measured body-frame direction, rotated into the
 * navigation frame by the value's rotation, aligns with the known navigation
 * frame reference direction:
 *   R * bMeasured == nRef
 *
 * @tparam VALUE Value type with a `rotation(OptionalJacobian<3, dim>)` method.
 * `Rot3` is supported as a special case.
 *
 * @ingroup navigation
 */
template <class VALUE>
class GTSAM_EXPORT AttitudeFactor : public NoiseModelFactorN<VALUE> {
 public:
  typedef AttitudeFactor<VALUE> This;
  typedef NoiseModelFactorN<VALUE> Base;

  using Base::evaluateError;

  /// shorthand for a smart pointer to a factor
  typedef std::shared_ptr<This> shared_ptr;

 protected:
  Unit3 nRef_, bMeasured_;

 public:
  /** default constructor - only use for serialization */
  AttitudeFactor() {}

  /**
   * @brief Constructor
   * @param key of the variable that will be constrained
   * @param nRef Reference direction in the navigation frame (e.g., gravity).
   * @param model Gaussian noise model
   * @param bMeasured Measured direction in the body frame (e.g., from IMU
   * accelerometer). Default is Unit3(0, 0, 1).
   */
  AttitudeFactor(Key key, const Unit3& nRef, const SharedNoiseModel& model,
                 const Unit3& bMeasured = Unit3(0, 0, 1))
      : Base(model, key), nRef_(nRef), bMeasured_(bMeasured) {}

  ~AttitudeFactor() override {}

  /** vector of errors */
  Vector attitudeError(const Rot3& nRb, OptionalJacobian<2, 3> H = {}) const {
    if (H) {
      Matrix23 D_nMeasured_R;
      const Unit3 nMeasured = nRb.rotate(bMeasured_, D_nMeasured_R);
      Matrix22 D_e_nMeasured;
      const Vector error = nRef_.errorVector(nMeasured, {}, D_e_nMeasured);
      *H = D_e_nMeasured * D_nMeasured_R;
      return error;
    } else {
      return nRef_.errorVector(nRb * bMeasured_);
    }
  }

  const Unit3& nRef() const { return nRef_; }
  const Unit3& bMeasured() const { return bMeasured_; }

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V43
  [[deprecated("Use nRef() instead")]]
  const Unit3& nZ() const {
    return nRef_;
  }

  [[deprecated("Use bMeasured() instead")]]
  const Unit3& bRef() const {
    return bMeasured_;
  }
#endif

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** print */
  void print(
      const std::string& s = "",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    std::cout << (s.empty() ? "" : s + " ")
              << detail::attitudeFactorName<VALUE>() << " on "
              << keyFormatter(this->key()) << "\n";
    nRef_.print("  reference direction in nav frame: ");
    bMeasured_.print("  measured direction in body frame: ");
    this->noiseModel_->print("  noise model: ");
  }

  /** equals */
  bool equals(const NonlinearFactor& expected,
              double tol = 1e-9) const override {
    const This* e = dynamic_cast<const This*>(&expected);
    return e != nullptr && Base::equals(*e, tol) &&
           nRef_.equals(e->nRef_, tol) && bMeasured_.equals(e->bMeasured_, tol);
  }

  /** vector of errors */
  Vector evaluateError(const VALUE& value,
                       OptionalMatrixType H) const override {
    if constexpr (std::is_same_v<VALUE, Rot3>) {
      return attitudeError(value, H);
    } else {
      if (H) {
        Matrix H_rotation_value;
        const Rot3 nRb = value.rotation(H_rotation_value);
        Matrix23 H_error_rotation;
        const Vector error = attitudeError(nRb, H_error_rotation);
        *H = H_error_rotation * H_rotation_value;
        return error;
      } else {
        return attitudeError(value.rotation());
      }
    }
  }

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp(
        "NoiseModelFactor1", boost::serialization::base_object<Base>(*this));
    ar& boost::serialization::make_nvp("nRef_", nRef_);
    ar& boost::serialization::make_nvp("bMeasured_", bMeasured_);
  }
#endif

 public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW
};

template <class VALUE>
struct traits<AttitudeFactor<VALUE>> : public Testable<AttitudeFactor<VALUE>> {
};

}  // namespace gtsam
