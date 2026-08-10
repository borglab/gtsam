/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file GeneralSFMFactor.h
 *
 * @brief a general SFM factor with an unknown calibration
 *
 * @date Dec 15, 2010
 * @author Kai Ni
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/base/Manifold.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/SymmetricBlockMatrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/VectorConstants.h>
#include <gtsam/base/concepts.h>
#include <gtsam/base/timing.h>
#include <gtsam/base/types.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/BinaryJacobianFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/TernaryJacobianFactor.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#if GTSAM_ENABLE_BOOST_SERIALIZATION
#include <boost/serialization/nvp.hpp>
#endif
#include <iostream>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace boost {
namespace serialization {
class access;
} /* namespace serialization */
} /* namespace boost */

namespace gtsam {

/**
 * Non-linear factor for a constraint derived from a 2D measurement.
 * The calibration is unknown here compared to GenericProjectionFactor
 * @ingroup slam
 */
template <class CAMERA, class LANDMARK>
class GeneralSFMFactor
    : public NoiseModelFactorT<
          typename traits<typename CAMERA::Measurement>::TangentVector, CAMERA,
          LANDMARK> {
  GTSAM_CONCEPT_MANIFOLD_TYPE(CAMERA)
  GTSAM_CONCEPT_MANIFOLD_TYPE(LANDMARK)

  using Measurement = typename CAMERA::Measurement;
  using ErrorVector = typename traits<Measurement>::TangentVector;
  static const int DimC = FixedDimension<CAMERA>::value;
  static const int DimL = FixedDimension<LANDMARK>::value;
  static const int ZDim = traits<Measurement>::dimension;
  typedef Eigen::Matrix<double, ZDim, DimC> JacobianC;
  typedef Eigen::Matrix<double, ZDim, DimL> JacobianL;

 protected:
  Measurement measured_;  ///< the measurement

 public:
  typedef GeneralSFMFactor<CAMERA, LANDMARK> This;  ///< typedef for this object
  typedef NoiseModelFactorT<ErrorVector, CAMERA, LANDMARK>
      Base;  ///< typedef for the base class

  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;

  // shorthand for a smart pointer to a factor
  typedef std::shared_ptr<This> shared_ptr;

  /**
   * Constructor
   * @param measured is the 2 dimensional measurement of point in image
   * @param model is the standard deviation of the measurements
   * @param cameraKey is the index of the camera
   * @param landmarkKey is the index of the landmark
   */
  GeneralSFMFactor(const Measurement& measured, const SharedNoiseModel& model,
                   Key cameraKey, Key landmarkKey)
      : Base(model, cameraKey, landmarkKey), measured_(measured) {}

  GeneralSFMFactor() : measured_(Measurement()) {}  ///< default constructor
  ///< constructor that takes a Point2 (only enabled for Point2 measurements)
  template <
      typename M = Measurement,
      typename std::enable_if<std::is_same<M, Point2>::value, int>::type = 0>
  GeneralSFMFactor(const Point2& p) : measured_(p) {}
  ///< constructor that takes doubles x,y to make a Point2
  template <
      typename M = Measurement,
      typename std::enable_if<std::is_same<M, Point2>::value, int>::type = 0>
  GeneralSFMFactor(double x, double y) : measured_(x, y) {}

  ~GeneralSFMFactor() override {}  ///< destructor

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter for printing out Symbols
   */
  void print(
      const std::string& s = "SFMFactor",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    Base::print(s, keyFormatter);
    traits<Measurement>::Print(measured_, s + ".z");
  }

  /**
   * equals
   */
  bool equals(const NonlinearFactor& p, double tol = 1e-9) const override {
    const This* e = dynamic_cast<const This*>(&p);
    return e && Base::equals(p, tol) &&
           traits<Measurement>::Equals(this->measured_, e->measured_, tol);
  }

  /** h(x)-z */
  ErrorVector evaluateError(const CAMERA& camera, const LANDMARK& point,
                            OptionalMatrixType H1,
                            OptionalMatrixType H2) const override {
    try {
      Measurement predicted = camera.project2(point, H1, H2);
      return traits<Measurement>::Local(measured_, predicted);
    } catch (CheiralityException& e [[maybe_unused]]) {
      if (H1) *H1 = JacobianC::Zero();
      if (H2) *H2 = JacobianL::Zero();
      // TODO Print the exception via logging
      return Vector::Zero(ZDim);
    }
  }

  /**
   * Error function with Eigen::Ref for zero-malloc linearization.
   * Writes Jacobians directly into the provided matrix blocks.
   */
  ErrorVector evaluateError(const CAMERA& camera, const LANDMARK& point,
                            Eigen::Ref<Matrix> H1,
                            Eigen::Ref<Matrix> H2) const {
    try {
      Measurement predicted = camera.project2(point, H1, H2);
      return traits<Measurement>::Local(measured_, predicted);
    } catch (CheiralityException& e [[maybe_unused]]) {
      H1.setZero();
      H2.setZero();
      return ErrorVector::Zero();
    }
  }

  /** Linearize using fixed-size Jacobians. */
  std::shared_ptr<GaussianFactor> linearize(
      const Values& values) const override {
    if (!this->active(values)) return std::shared_ptr<JacobianFactor>();

    const Key key1 = this->key1(), key2 = this->key2();
    JacobianC Dcamera;
    JacobianL Dlandmark;
    ErrorVector b =
        -evaluateError(values.at<CAMERA>(key1), values.at<LANDMARK>(key2),
                       Dcamera, Dlandmark);

    const SharedNoiseModel& noiseModel = this->noiseModel();
    if (noiseModel && static_cast<size_t>(ZDim) != noiseModel->dim()) {
      throw std::invalid_argument(
          "NoiseModelFactor: NoiseModel has dimension " +
          std::to_string(noiseModel->dim()) + " instead of " +
          std::to_string(ZDim) + ".");
    }

    if (noiseModel && !noiseModel->isUnit()) {
      Matrix dynamicCamera = Dcamera, dynamicLandmark = Dlandmark;
      Vector dynamicB = b;
      noiseModel->WhitenSystem(dynamicCamera, dynamicLandmark, dynamicB);
      Dcamera = dynamicCamera;
      Dlandmark = dynamicLandmark;
      b = dynamicB;
    }

    SharedDiagonal linearModel;
    if (noiseModel && noiseModel->isConstrained()) {
      const auto constrained =
          std::static_pointer_cast<noiseModel::Constrained>(noiseModel);
      linearModel = constrained->unit();
    }

    return std::make_shared<BinaryJacobianFactor<ZDim, DimC, DimL>>(
        key1, Dcamera, key2, Dlandmark, b, linearModel);
  }

  /** return the measured */
  inline const Measurement measured() const { return measured_; }

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/) {
    // NoiseModelFactor2 instead of NoiseModelFactorN for backward compatibility
    ar& boost::serialization::make_nvp(
        "NoiseModelFactor2", boost::serialization::base_object<Base>(*this));
    ar& BOOST_SERIALIZATION_NVP(measured_);
  }
#endif
};

template <class CAMERA, class LANDMARK>
struct traits<GeneralSFMFactor<CAMERA, LANDMARK>>
    : Testable<GeneralSFMFactor<CAMERA, LANDMARK>> {};

/**
 * Non-linear factor for a constraint derived from a 2D measurement.
 * Compared to GeneralSFMFactor, it is a ternary-factor because the calibration
 * is isolated from camera..
 */
template <class CALIBRATION>
class GeneralSFMFactor2
    : public NoiseModelFactorT<Vector2, Pose3, Point3, CALIBRATION> {
  GTSAM_CONCEPT_MANIFOLD_TYPE(CALIBRATION)
  static const int DimK = FixedDimension<CALIBRATION>::value;

 protected:
  Point2 measured_;  ///< the 2D measurement

 public:
  typedef GeneralSFMFactor2<CALIBRATION> This;
  typedef PinholeCamera<CALIBRATION> Camera;  ///< typedef for camera type
  typedef NoiseModelFactorT<Vector2, Pose3, Point3, CALIBRATION>
      Base;  ///< typedef for the base class

  // shorthand for a smart pointer to a factor
  typedef std::shared_ptr<This> shared_ptr;

  /**
   * Constructor
   * @param measured is the 2 dimensional measurement of point in image
   * @param model is the standard deviation of the measurements
   * @param poseKey is the index of the camera
   * @param landmarkKey is the index of the landmark
   * @param calibKey is the index of the calibration
   */
  GeneralSFMFactor2(const Point2& measured, const SharedNoiseModel& model,
                    Key poseKey, Key landmarkKey, Key calibKey)
      : Base(model, poseKey, landmarkKey, calibKey), measured_(measured) {}
  GeneralSFMFactor2() : measured_(0.0, 0.0) {}  ///< default constructor

  ~GeneralSFMFactor2() override {}  ///< destructor

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(
      const std::string& s = "SFMFactor2",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    Base::print(s, keyFormatter);
    traits<Point2>::Print(measured_, s + ".z");
  }

  /**
   * equals
   */
  bool equals(const NonlinearFactor& p, double tol = 1e-9) const override {
    const This* e = dynamic_cast<const This*>(&p);
    return e && Base::equals(p, tol) &&
           traits<Point2>::Equals(this->measured_, e->measured_, tol);
  }

  /** h(x)-z */
  Vector2 evaluateError(const Pose3& pose3, const Point3& point,
                        const CALIBRATION& calib, OptionalMatrixType H1,
                        OptionalMatrixType H2,
                        OptionalMatrixType H3) const override {
    try {
      Camera camera(pose3, calib);
      return camera.project(point, H1, H2, H3) - measured_;
    } catch (CheiralityException& e) {
      if (H1) *H1 = Matrix::Zero(2, 6);
      if (H2) *H2 = Matrix::Zero(2, 3);
      if (H3) *H3 = Matrix::Zero(2, DimK);
      std::cout << e.what() << ": Landmark "
                << DefaultKeyFormatter(this->key2()) << " behind Camera "
                << DefaultKeyFormatter(this->key1()) << std::endl;
    }
    return Z_2x1;
  }

  /** return the measured */
  inline const Point2 measured() const { return measured_; }

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/) {
    // NoiseModelFactor3 instead of NoiseModelFactorN for backward compatibility
    ar& boost::serialization::make_nvp(
        "NoiseModelFactor3", boost::serialization::base_object<Base>(*this));
    ar& BOOST_SERIALIZATION_NVP(measured_);
  }
#endif
};

template <class CALIBRATION>
struct traits<GeneralSFMFactor2<CALIBRATION>>
    : Testable<GeneralSFMFactor2<CALIBRATION>> {};

}  // namespace gtsam
