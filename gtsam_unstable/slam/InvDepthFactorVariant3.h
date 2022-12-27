
/**
 * @file InvDepthFactorVariant3.h
 * @brief Inverse Depth Factor based on Civera09tro, Montiel06rss.
 * Landmarks are parameterized as (theta,phi,rho). The factor involves
 * two poses and a landmark. The first pose is the reference frame
 * from which (theta, phi, rho) is measured.
 * @author Chris Beall, Stephen Williams
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/base/numericalDerivative.h>

#include <boost/bind/bind.hpp>

namespace gtsam {

/**
 * Binary factor representing the first visual measurement using an inverse-depth parameterization
 */
class InvDepthFactorVariant3a: public NoiseModelFactorN<Pose3, Vector3> {
  ADD_NOISE_MODEL_FACTOR_N_DEPRECATED_TYPEDEFS(InvDepthFactorVariant3a, 2);

protected:

  // Keep a copy of measurement and calibration for I/O
  Point2 measured_;        ///< 2D measurement
  Cal3_S2::shared_ptr K_;  ///< shared pointer to calibration object

public:

  /// shorthand for base class type
  typedef NoiseModelFactorN<Pose3, Vector3> Base;

  /// shorthand for this class
  typedef InvDepthFactorVariant3a This;

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /// Default constructor
  InvDepthFactorVariant3a() :
      measured_(0.0, 0.0), K_(new Cal3_S2(444, 555, 666, 777, 888)) {
  }

  /**
   * Constructor
   * TODO: Mark argument order standard (keys, measurement, parameters)
   * @param measured is the 2 dimensional location of point in image (the measurement)
   * @param model is the standard deviation
   * @param poseKey is the index of the camera pose
   * @param pointKey is the index of the landmark
   * @param invDepthKey is the index of inverse depth
   * @param K shared pointer to the constant calibration
   */
  InvDepthFactorVariant3a(const Key poseKey, const Key landmarkKey,
      const Point2& measured, const Cal3_S2::shared_ptr& K, const SharedNoiseModel& model) :
        Base(model, poseKey, landmarkKey), measured_(measured), K_(K) {}

  /** Virtual destructor */
  ~InvDepthFactorVariant3a() override {}

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(const std::string& s = "InvDepthFactorVariant3a",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    Base::print(s, keyFormatter);
    traits<Point2>::Print(measured_, s + ".z");
  }

  /// equals
  bool equals(const NonlinearFactor& p, double tol = 1e-9) const override {
    const This *e = dynamic_cast<const This*>(&p);
    return e
        && Base::equals(p, tol)
        && traits<Point2>::Equals(this->measured_, e->measured_, tol)
        && this->K_->equals(*e->K_, tol);
  }

  Vector inverseDepthError(const Pose3& pose, const Vector3& landmark) const {
    try {
      // Calculate the 3D coordinates of the landmark in the Pose frame
      double theta = landmark(0), phi = landmark(1), rho = landmark(2);
      Point3 pose_P_landmark(cos(phi)*sin(theta)/rho, sin(phi)/rho, cos(phi)*cos(theta)/rho);
      // Convert the landmark to world coordinates
      Point3 world_P_landmark = pose.transformFrom(pose_P_landmark);
      // Project landmark into Pose2
      PinholeCamera<Cal3_S2> camera(pose, *K_);
      return camera.project(world_P_landmark) - measured_;
    } catch( CheiralityException& e) {
      std::cout << e.what()
          << ": Inverse Depth Landmark [" << DefaultKeyFormatter(this->key<1>()) << "," << DefaultKeyFormatter(this->key<2>()) << "]"
          << " moved behind camera [" << DefaultKeyFormatter(this->key<1>()) << "]"
          << std::endl;
      return Vector::Ones(2) * 2.0 * K_->fx();
    }
    return (Vector(1) << 0.0).finished();
  }

  /// Evaluate error h(x)-z and optionally derivatives
  Vector evaluateError(const Pose3& pose, const Vector3& landmark,
      boost::optional<Matrix&> H1=boost::none,
      boost::optional<Matrix&> H2=boost::none) const override {

    if(H1) {
      (*H1) = numericalDerivative11<Vector, Pose3>(
          std::bind(&InvDepthFactorVariant3a::inverseDepthError, this,
                      std::placeholders::_1, landmark),
          pose);
    }
    if(H2) {
      (*H2) = numericalDerivative11<Vector, Vector3>(
          std::bind(&InvDepthFactorVariant3a::inverseDepthError, this, pose,
                      std::placeholders::_1),
          landmark);
    }

    return inverseDepthError(pose, landmark);
  }

  /** return the measurement */
  const Point2& imagePoint() const {
    return measured_;
  }

  /** return the calibration object */
  const Cal3_S2::shared_ptr calibration() const {
    return K_;
  }

private:

  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar & BOOST_SERIALIZATION_NVP(measured_);
    ar & BOOST_SERIALIZATION_NVP(K_);
  }
};

/**
 * Ternary factor representing a visual measurement using an inverse-depth parameterization
 */
class InvDepthFactorVariant3b: public NoiseModelFactorN<Pose3, Pose3, Vector3> {
  ADD_NOISE_MODEL_FACTOR_N_DEPRECATED_TYPEDEFS(InvDepthFactorVariant3b, 3);

protected:

  // Keep a copy of measurement and calibration for I/O
  Point2 measured_;        ///< 2D measurement
  Cal3_S2::shared_ptr K_;  ///< shared pointer to calibration object

public:

  /// shorthand for base class type
  typedef NoiseModelFactorN<Pose3, Pose3, Vector3> Base;

  /// shorthand for this class
  typedef InvDepthFactorVariant3b This;

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /// Default constructor
  InvDepthFactorVariant3b() :
      measured_(0.0, 0.0), K_(new Cal3_S2(444, 555, 666, 777, 888)) {
  }

  /**
   * Constructor
   * TODO: Mark argument order standard (keys, measurement, parameters)
   * @param measured is the 2 dimensional location of point in image (the measurement)
   * @param model is the standard deviation
   * @param poseKey is the index of the camera pose
   * @param pointKey is the index of the landmark
   * @param invDepthKey is the index of inverse depth
   * @param K shared pointer to the constant calibration
   */
  InvDepthFactorVariant3b(const Key poseKey1, const Key poseKey2, const Key landmarkKey,
      const Point2& measured, const Cal3_S2::shared_ptr& K, const SharedNoiseModel& model) :
        Base(model, poseKey1, poseKey2, landmarkKey), measured_(measured), K_(K) {}

  /** Virtual destructor */
  ~InvDepthFactorVariant3b() override {}

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(const std::string& s = "InvDepthFactorVariant3",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    Base::print(s, keyFormatter);
    traits<Point2>::Print(measured_, s + ".z");
  }

  /// equals
  bool equals(const NonlinearFactor& p, double tol = 1e-9) const override {
    const This *e = dynamic_cast<const This*>(&p);
    return e
        && Base::equals(p, tol)
        && traits<Point2>::Equals(this->measured_, e->measured_, tol)
        && this->K_->equals(*e->K_, tol);
  }

  Vector inverseDepthError(const Pose3& pose1, const Pose3& pose2, const Vector3& landmark) const {
    try {
      // Calculate the 3D coordinates of the landmark in the Pose1 frame
      double theta = landmark(0), phi = landmark(1), rho = landmark(2);
      Point3 pose1_P_landmark(cos(phi)*sin(theta)/rho, sin(phi)/rho, cos(phi)*cos(theta)/rho);
      // Convert the landmark to world coordinates
      Point3 world_P_landmark = pose1.transformFrom(pose1_P_landmark);
      // Project landmark into Pose2
      PinholeCamera<Cal3_S2> camera(pose2, *K_);
      return camera.project(world_P_landmark) - measured_;
    } catch( CheiralityException& e) {
      std::cout << e.what()
          << ": Inverse Depth Landmark [" << DefaultKeyFormatter(this->key<1>()) << "," << DefaultKeyFormatter(this->key<3>()) << "]"
          << " moved behind camera " << DefaultKeyFormatter(this->key<2>())
          << std::endl;
      return Vector::Ones(2) * 2.0 * K_->fx();
    }
    return (Vector(1) << 0.0).finished();
  }

  /// Evaluate error h(x)-z and optionally derivatives
  Vector evaluateError(const Pose3& pose1, const Pose3& pose2, const Vector3& landmark,
      boost::optional<Matrix&> H1=boost::none,
      boost::optional<Matrix&> H2=boost::none,
      boost::optional<Matrix&> H3=boost::none) const override {

    if(H1)
      (*H1) = numericalDerivative11<Vector, Pose3>(
          std::bind(&InvDepthFactorVariant3b::inverseDepthError, this,
                      std::placeholders::_1, pose2, landmark),
          pose1);

    if(H2)
      (*H2) = numericalDerivative11<Vector, Pose3>(
          std::bind(&InvDepthFactorVariant3b::inverseDepthError, this, pose1,
                      std::placeholders::_1, landmark),
          pose2);

    if(H3)
      (*H3) = numericalDerivative11<Vector, Vector3>(
          std::bind(&InvDepthFactorVariant3b::inverseDepthError, this, pose1,
                      pose2, std::placeholders::_1),
          landmark);

    return inverseDepthError(pose1, pose2, landmark);
  }

  /** return the measurement */
  const Point2& imagePoint() const {
    return measured_;
  }

  /** return the calibration object */
  const Cal3_S2::shared_ptr calibration() const {
    return K_;
  }

private:

  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar & BOOST_SERIALIZATION_NVP(measured_);
    ar & BOOST_SERIALIZATION_NVP(K_);
  }
};

} // \ namespace gtsam
