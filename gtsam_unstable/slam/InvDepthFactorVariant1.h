
/**
 * @file InvDepthFactorVariant1.h
 * @brief Inverse Depth Factor based on Civera09tro, Montiel06rss.
 * Landmarks are parameterized as (x,y,z,theta,phi,rho). The factor involves
 * a single pose and a landmark. The landmark parameterization contains the
 * reference point internally (and will thus be updated as well during optimization).
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
 * Binary factor representing a visual measurement using an inverse-depth parameterization
 */
class InvDepthFactorVariant1: public NoiseModelFactorN<Pose3, Vector6> {
  ADD_NOISE_MODEL_FACTOR_N_DEPRECATED_TYPEDEFS(InvDepthFactorVariant1, 2);

protected:

  // Keep a copy of measurement and calibration for I/O
  Point2 measured_;        ///< 2D measurement
  Cal3_S2::shared_ptr K_;  ///< shared pointer to calibration object

public:

  /// shorthand for base class type
  typedef NoiseModelFactorN<Pose3, Vector6> Base;

  /// shorthand for this class
  typedef InvDepthFactorVariant1 This;

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /// Default constructor
  InvDepthFactorVariant1() :
      measured_(0.0, 0.0), K_(new Cal3_S2(444, 555, 666, 777, 888)) {
  }

  /**
   * Constructor
   * @param poseKey is the index of the camera pose
   * @param pointKey is the index of the landmark
   * @param measured is the 2 dimensional location of point in image (the measurement)
   * @param K shared pointer to the constant calibration
   * @param model is the standard deviation
   */
  InvDepthFactorVariant1(const Key poseKey, const Key landmarkKey,
      const Point2& measured, const Cal3_S2::shared_ptr& K, const SharedNoiseModel& model) :
        Base(model, poseKey, landmarkKey), measured_(measured), K_(K) {}

  /** Virtual destructor */
  ~InvDepthFactorVariant1() override {}

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(const std::string& s = "InvDepthFactorVariant1",
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

  Vector inverseDepthError(const Pose3& pose, const Vector6& landmark) const {
    try {
      // Calculate the 3D coordinates of the landmark in the world frame
      double x = landmark(0), y = landmark(1), z = landmark(2);
      double theta = landmark(3), phi = landmark(4), rho = landmark(5);
      Point3 world_P_landmark = Point3(x, y, z) + Point3(cos(theta)*cos(phi)/rho, sin(theta)*cos(phi)/rho, sin(phi)/rho);
      // Project landmark into Pose2
      PinholeCamera<Cal3_S2> camera(pose, *K_);
      return camera.project(world_P_landmark) - measured_;
    } catch( CheiralityException& e) {
      std::cout << e.what()
          << ": Inverse Depth Landmark [" << DefaultKeyFormatter(this->key<2>()) << "]"
          << " moved behind camera [" << DefaultKeyFormatter(this->key<1>()) <<"]"
          << std::endl;
      return Vector::Ones(2) * 2.0 * K_->fx();
    }
    return (Vector(1) << 0.0).finished();
  }

  /// Evaluate error h(x)-z and optionally derivatives
  Vector evaluateError(const Pose3& pose, const Vector6& landmark,
      boost::optional<Matrix&> H1=boost::none,
      boost::optional<Matrix&> H2=boost::none) const override {

    if (H1) {
      (*H1) = numericalDerivative11<Vector, Pose3>(
          std::bind(&InvDepthFactorVariant1::inverseDepthError, this,
                      std::placeholders::_1, landmark),
          pose);
    }
    if (H2) {
      (*H2) = numericalDerivative11<Vector, Vector6>(
          std::bind(&InvDepthFactorVariant1::inverseDepthError, this, pose,
              std::placeholders::_1), landmark);
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

} // \ namespace gtsam
