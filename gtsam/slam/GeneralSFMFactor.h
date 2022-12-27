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
 */

#pragma once

#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/linear/BinaryJacobianFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/base/concepts.h>
#include <gtsam/base/Manifold.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/SymmetricBlockMatrix.h>
#include <gtsam/base/types.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/timing.h>

#include <boost/none.hpp>
#include <boost/optional/optional.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <iostream>
#include <string>

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
template<class CAMERA, class LANDMARK>
class GeneralSFMFactor: public NoiseModelFactorN<CAMERA, LANDMARK> {
  ADD_NOISE_MODEL_FACTOR_N_DEPRECATED_TYPEDEFS(GeneralSFMFactor, 2);

  GTSAM_CONCEPT_MANIFOLD_TYPE(CAMERA)
  GTSAM_CONCEPT_MANIFOLD_TYPE(LANDMARK)

  static const int DimC = FixedDimension<CAMERA>::value;
  static const int DimL = FixedDimension<LANDMARK>::value;
  typedef Eigen::Matrix<double, 2, DimC> JacobianC;
  typedef Eigen::Matrix<double, 2, DimL> JacobianL;

protected:

  Point2 measured_; ///< the 2D measurement

public:

  typedef GeneralSFMFactor<CAMERA, LANDMARK> This;///< typedef for this object
  typedef NoiseModelFactorN<CAMERA, LANDMARK> Base;///< typedef for the base class

  // shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /**
   * Constructor
   * @param measured is the 2 dimensional location of point in image (the measurement)
   * @param model is the standard deviation of the measurements
   * @param cameraKey is the index of the camera
   * @param landmarkKey is the index of the landmark
   */
  GeneralSFMFactor(const Point2& measured, const SharedNoiseModel& model,
                   Key cameraKey, Key landmarkKey)
      : Base(model, cameraKey, landmarkKey), measured_(measured) {}

  GeneralSFMFactor() : measured_(0.0, 0.0) {}  ///< default constructor
  ///< constructor that takes a Point2
  GeneralSFMFactor(const Point2& p) : measured_(p) {}
  ///< constructor that takes doubles x,y to make a Point2
  GeneralSFMFactor(double x, double y) : measured_(x, y) {}

  ~GeneralSFMFactor() override {} ///< destructor

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));}

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter for printing out Symbols
   */
  void print(const std::string& s = "SFMFactor", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    Base::print(s, keyFormatter);
    traits<Point2>::Print(measured_, s + ".z");
  }

  /**
   * equals
   */
  bool equals(const NonlinearFactor &p, double tol = 1e-9) const override {
    const This* e = dynamic_cast<const This*>(&p);
    return e && Base::equals(p, tol) && traits<Point2>::Equals(this->measured_, e->measured_, tol);
  }

  /** h(x)-z */
  Vector evaluateError(const CAMERA& camera, const LANDMARK& point,
      boost::optional<Matrix&> H1=boost::none, boost::optional<Matrix&> H2=boost::none) const override {
    try {
      return camera.project2(point,H1,H2) - measured_;
    }
    catch( CheiralityException& e) {
      if (H1) *H1 = JacobianC::Zero();
      if (H2) *H2 = JacobianL::Zero();
      //TODO Print the exception via logging
      return Z_2x1;
    }
  }

  /// Linearize using fixed-size matrices
  boost::shared_ptr<GaussianFactor> linearize(const Values& values) const override {
    // Only linearize if the factor is active
    if (!this->active(values)) return boost::shared_ptr<JacobianFactor>();

    const Key key1 = this->template key<1>(), key2 = this->template key<2>();
    JacobianC H1;
    JacobianL H2;
    Vector2 b;
    try {
      const CAMERA& camera = values.at<CAMERA>(key1);
      const LANDMARK& point = values.at<LANDMARK>(key2);
      b = measured() - camera.project2(point, H1, H2);
    } catch (CheiralityException& e) {
      H1.setZero();
      H2.setZero();
      b.setZero();
      //TODO Print the exception via logging
    }

    // Whiten the system if needed
    const SharedNoiseModel& noiseModel = this->noiseModel();
    if (noiseModel && !noiseModel->isUnit()) {
      // TODO: implement WhitenSystem for fixed size matrices and include
      // above
      H1 = noiseModel->Whiten(H1);
      H2 = noiseModel->Whiten(H2);
      b = noiseModel->Whiten(b);
    }

    // Create new (unit) noiseModel, preserving constraints if applicable
    SharedDiagonal model;
    if (noiseModel && noiseModel->isConstrained()) {
      model = boost::static_pointer_cast<noiseModel::Constrained>(noiseModel)->unit();
    }

    return boost::make_shared<BinaryJacobianFactor<2, DimC, DimL> >(key1, H1, key2, H2, b, model);
  }

  /** return the measured */
  inline const Point2 measured() const {
    return measured_;
  }

private:
  /** Serialization function */
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int /*version*/) {
    // NoiseModelFactor2 instead of NoiseModelFactorN for backward compatibility
    ar & boost::serialization::make_nvp("NoiseModelFactor2",
        boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(measured_);
  }
};

template<class CAMERA, class LANDMARK>
struct traits<GeneralSFMFactor<CAMERA, LANDMARK> > : Testable<
    GeneralSFMFactor<CAMERA, LANDMARK> > {
};

/**
 * Non-linear factor for a constraint derived from a 2D measurement.
 * Compared to GeneralSFMFactor, it is a ternary-factor because the calibration is isolated from camera..
 */
template<class CALIBRATION>
class GeneralSFMFactor2: public NoiseModelFactorN<Pose3, Point3, CALIBRATION> {
  ADD_NOISE_MODEL_FACTOR_N_DEPRECATED_TYPEDEFS(GeneralSFMFactor2, 3);

  GTSAM_CONCEPT_MANIFOLD_TYPE(CALIBRATION)
  static const int DimK = FixedDimension<CALIBRATION>::value;

protected:

  Point2 measured_; ///< the 2D measurement

public:

  typedef GeneralSFMFactor2<CALIBRATION> This;
  typedef PinholeCamera<CALIBRATION> Camera;///< typedef for camera type
  typedef NoiseModelFactorN<Pose3, Point3, CALIBRATION> Base;///< typedef for the base class

  // shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /**
   * Constructor
   * @param measured is the 2 dimensional location of point in image (the measurement)
   * @param model is the standard deviation of the measurements
   * @param poseKey is the index of the camera
   * @param landmarkKey is the index of the landmark
   * @param calibKey is the index of the calibration
   */
  GeneralSFMFactor2(const Point2& measured, const SharedNoiseModel& model, Key poseKey, Key landmarkKey, Key calibKey) :
  Base(model, poseKey, landmarkKey, calibKey), measured_(measured) {}
  GeneralSFMFactor2():measured_(0.0,0.0) {} ///< default constructor

  ~GeneralSFMFactor2() override {} ///< destructor

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));}

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(const std::string& s = "SFMFactor2", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    Base::print(s, keyFormatter);
    traits<Point2>::Print(measured_, s + ".z");
  }

  /**
   * equals
   */
  bool equals(const NonlinearFactor &p, double tol = 1e-9) const override {
    const This* e = dynamic_cast<const This*>(&p);
    return e && Base::equals(p, tol) && traits<Point2>::Equals(this->measured_, e->measured_, tol);
  }

  /** h(x)-z */
  Vector evaluateError(const Pose3& pose3, const Point3& point, const CALIBRATION &calib,
      boost::optional<Matrix&> H1=boost::none,
      boost::optional<Matrix&> H2=boost::none,
      boost::optional<Matrix&> H3=boost::none) const override
  {
    try {
      Camera camera(pose3,calib);
      return camera.project(point, H1, H2, H3) - measured_;
    }
    catch( CheiralityException& e) {
      if (H1) *H1 = Matrix::Zero(2, 6);
      if (H2) *H2 = Matrix::Zero(2, 3);
      if (H3) *H3 = Matrix::Zero(2, DimK);
      std::cout << e.what() << ": Landmark "<< DefaultKeyFormatter(this->template key<2>())
      << " behind Camera " << DefaultKeyFormatter(this->template key<1>()) << std::endl;
    }
    return Z_2x1;
  }

  /** return the measured */
  inline const Point2 measured() const {
    return measured_;
  }

private:
  /** Serialization function */
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int /*version*/) {
    // NoiseModelFactor3 instead of NoiseModelFactorN for backward compatibility
    ar & boost::serialization::make_nvp("NoiseModelFactor3",
        boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(measured_);
  }
};

template<class CALIBRATION>
struct traits<GeneralSFMFactor2<CALIBRATION> > : Testable<
    GeneralSFMFactor2<CALIBRATION> > {
};

} //namespace
