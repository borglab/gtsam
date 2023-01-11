
/**
 * @file InvDepthFactor3.h
 * @brief Inverse Depth Factor based on Civera09tro, Montiel06rss.
 * Landmarks are initialized from the first camera observation with
 * (x,y,z,theta,phi,inv_depth), where x,y,z are the coordinates of
 * the camera. InvDepthCamera provides methods to initialize inverse
 * depth landmarks (backproject), and to convert inverse depth
 * landmarks to cartesian coordinates (Point3) for visualization, etc.
 * The inverse depth parameterization is split into (x,y,z,theta,phi),
 * (inv_depth) to make it easy to add a prior on inverse depth alone
 * @author Chris Beall
 */

#pragma once

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam_unstable/geometry/InvDepthCamera3.h>

namespace gtsam {

/**
 * Ternary factor representing a visual measurement that includes inverse depth
 */
template<class POSE, class LANDMARK, class INVDEPTH>
class InvDepthFactor3: public NoiseModelFactorN<POSE, LANDMARK, INVDEPTH> {
protected:

  // Keep a copy of measurement and calibration for I/O
  Point2 measured_;                ///< 2D measurement
  boost::shared_ptr<Cal3_S2> K_;  ///< shared pointer to calibration object

public:

  /// shorthand for base class type
  typedef NoiseModelFactor3<POSE, LANDMARK, INVDEPTH> Base;
  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;


  /// shorthand for this class
  typedef InvDepthFactor3<POSE, LANDMARK, INVDEPTH> This;

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /// Default constructor
  InvDepthFactor3() :
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
  InvDepthFactor3(const Point2& measured, const SharedNoiseModel& model,
      const Key poseKey, Key pointKey, Key invDepthKey, const Cal3_S2::shared_ptr& K) :
        Base(model, poseKey, pointKey, invDepthKey), measured_(measured), K_(K) {}

  /** Virtual destructor */
  ~InvDepthFactor3() override {}

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(const std::string& s = "InvDepthFactor3",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    Base::print(s, keyFormatter);
    traits<Point2>::Print(measured_, s + ".z");
  }

  /// equals
  bool equals(const NonlinearFactor& p, double tol = 1e-9) const override {
    const This *e = dynamic_cast<const This*>(&p);
    return e && Base::equals(p, tol) && traits<Point2>::Equals(this->measured_, e->measured_, tol) && this->K_->equals(*e->K_, tol);
  }

  /// Evaluate error h(x)-z and optionally derivatives
  Vector evaluateError(const POSE& pose, const Vector5& point, const INVDEPTH& invDepth,
      OptionalMatrixType H1, OptionalMatrixType H2, OptionalMatrixType H3) const override {
    try {
      InvDepthCamera3<Cal3_S2> camera(pose, K_);
      return camera.project(point, invDepth, H1, H2, H3) - measured_;
    } catch( CheiralityException& e) {
      if (H1) *H1 = Matrix::Zero(2,6);
      if (H2) *H2 = Matrix::Zero(2,5);
      if (H3) *H3 = Matrix::Zero(2,1);
      std::cout << e.what() << ": Landmark "<< DefaultKeyFormatter(this->key2()) <<
          " moved behind camera " << DefaultKeyFormatter(this->key1()) << std::endl;
      return Vector::Ones(2) * 2.0 * K_->fx();
    }
    return (Vector(1) << 0.0).finished();
  }

  /** return the measurement */
  const Point2& imagePoint() const {
    return measured_;
  }

  /** return the calibration object */
  inline const Cal3_S2::shared_ptr calibration() const {
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
