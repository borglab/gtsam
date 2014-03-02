/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * testTriangulationFactor.h
 * @date March 2, 2014
 * @author Frank Dellaert
 */

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/base/numericalDerivative.h>
#include <boost/optional.hpp>

namespace gtsam {

/**
 * Non-linear factor for a constraint derived from a 2D measurement.
 * The calibration and pose are assumed known.
 * @addtogroup SLAM
 */
template<class CALIBRATION = Cal3_S2>
class TriangulationFactor: public NoiseModelFactor1<Point3> {

public:

  /// Camera type
  typedef PinholeCamera<CALIBRATION> Camera;

protected:

  // Keep a copy of measurement and calibration for I/O
  const Camera camera_; ///< Camera in which this landmark was seen
  const Point2 measured_; ///< 2D measurement

  // verbosity handling for Cheirality Exceptions
  const bool throwCheirality_; ///< If true, rethrows Cheirality exceptions (default: false)
  const bool verboseCheirality_; ///< If true, prints text for Cheirality exceptions (default: false)

public:

  /// shorthand for base class type
  typedef NoiseModelFactor1<Point3> Base;

  /// shorthand for this class
  typedef TriangulationFactor<CALIBRATION> This;

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /// Default constructor
  TriangulationFactor() :
      throwCheirality_(false), verboseCheirality_(false) {
  }

  /**
   * Constructor with exception-handling flags
   * @param camera is the camera in which unknown landmark is seen
   * @param measured is the 2 dimensional location of point in image (the measurement)
   * @param model is the standard deviation
   * @param pointKey is the index of the landmark
   * @param throwCheirality determines whether Cheirality exceptions are rethrown
   * @param verboseCheirality determines whether exceptions are printed for Cheirality
   */
  TriangulationFactor(const Camera& camera, const Point2& measured,
      const SharedNoiseModel& model, Key pointKey, bool throwCheirality = false,
      bool verboseCheirality = false) :
      Base(model, pointKey), camera_(camera), measured_(measured), throwCheirality_(
          throwCheirality), verboseCheirality_(verboseCheirality) {
  }

  /** Virtual destructor */
  virtual ~TriangulationFactor() {
  }

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
      DefaultKeyFormatter) const {
    std::cout << s << "TriangulationFactor,";
    camera_.print("camera");
    measured_.print("z");
    Base::print("", keyFormatter);
  }

  /// equals
  virtual bool equals(const NonlinearFactor& p, double tol = 1e-9) const {
    const This *e = dynamic_cast<const This*>(&p);
    return e && Base::equals(p, tol) && this->camera_.equals(e->camera_, tol)
        && this->measured_.equals(e->measured_, tol);
  }

  /// Evaluate error h(x)-z and optionally derivatives
  Vector evaluateError(const Point3& point, boost::optional<Matrix&> H2 =
      boost::none) const {
    try {
      Point2 error(camera_.project(point, boost::none, H2) - measured_);
      return error.vector();
    } catch (CheiralityException& e) {
      if (H2)
        *H2 = zeros(2, 3);
      if (verboseCheirality_)
        std::cout << e.what() << ": Landmark "
            << DefaultKeyFormatter(this->key()) << " moved behind camera"
            << std::endl;
      if (throwCheirality_)
        throw e;
      return ones(2) * 2.0 * camera_.calibration().fx();
    }
  }

  /** return the measurement */
  const Point2& measured() const {
    return measured_;
  }

  /** return verbosity */
  inline bool verboseCheirality() const {
    return verboseCheirality_;
  }

  /** return flag for throwing cheirality exceptions */
  inline bool throwCheirality() const {
    return throwCheirality_;
  }

private:

  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar & BOOST_SERIALIZATION_NVP(camera_);
    ar & BOOST_SERIALIZATION_NVP(measured_);
    ar & BOOST_SERIALIZATION_NVP(throwCheirality_);
    ar & BOOST_SERIALIZATION_NVP(verboseCheirality_);
  }
};
} // \ namespace gtsam


