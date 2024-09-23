/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * triangulationFactor.h
 * @date March 2, 2014
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/CalibratedCamera.h>

namespace gtsam {

/**
 * Non-linear factor for a constraint derived from a 2D measurement.
 * The calibration and pose are assumed known.
 * @ingroup slam
 */
template<class CAMERA>
class TriangulationFactor: public NoiseModelFactorN<Point3> {

public:

  /// CAMERA type
  using Camera = CAMERA;

protected:

  /// shorthand for base class type
  using Base = NoiseModelFactorN<Point3>;

  /// shorthand for this class
  using This = TriangulationFactor<CAMERA>;

  /// shorthand for measurement type, e.g. Point2 or StereoPoint2
  using Measurement = typename CAMERA::Measurement;

  // Keep a copy of measurement and calibration for I/O
  const CAMERA camera_; ///< CAMERA in which this landmark was seen
  const Measurement measured_; ///< 2D measurement

  // verbosity handling for Cheirality Exceptions
  const bool throwCheirality_; ///< If true, rethrows Cheirality exceptions (default: false)
  const bool verboseCheirality_; ///< If true, prints text for Cheirality exceptions (default: false)

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// shorthand for a smart pointer to a factor
  using shared_ptr = std::shared_ptr<This>;

  // Provide access to the Matrix& version of evaluateError:
  using NoiseModelFactor1<Point3>::evaluateError;

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
  TriangulationFactor(const CAMERA& camera, const Measurement& measured,
      const SharedNoiseModel& model, Key pointKey, bool throwCheirality = false,
      bool verboseCheirality = false) :
      Base(model, pointKey), camera_(camera), measured_(measured), throwCheirality_(
          throwCheirality), verboseCheirality_(verboseCheirality) {
    if (model && model->dim() != traits<Measurement>::dimension)
      throw std::invalid_argument(
          "TriangulationFactor must be created with "
              + std::to_string((int) traits<Measurement>::dimension)
              + "-dimensional noise model.");
  }

  /** Virtual destructor */
  ~TriangulationFactor() override {
  }

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
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
      DefaultKeyFormatter) const override {
    std::cout << s << "TriangulationFactor,";
    camera_.print("camera");
    traits<Measurement>::Print(measured_, "z");
    Base::print("", keyFormatter);
  }

  /// equals
  bool equals(const NonlinearFactor& p, double tol = 1e-9) const override {
    const This *e = dynamic_cast<const This*>(&p);
    return e && Base::equals(p, tol) && this->camera_.equals(e->camera_, tol)
        && traits<Measurement>::Equals(this->measured_, e->measured_, tol);
  }

  /// Evaluate error h(x)-z and optionally derivatives
  Vector evaluateError(const Point3& point, OptionalMatrixType H2) const override {
    try {
      return traits<Measurement>::Local(measured_, camera_.project2(point, OptionalNone, H2));
    } catch (CheiralityException& e) {
      if (H2)
        *H2 = Matrix::Zero(traits<Measurement>::dimension, 3);
      if (verboseCheirality_)
        std::cout << e.what() << ": Landmark "
            << DefaultKeyFormatter(this->key()) << " moved behind camera"
            << std::endl;
      if (throwCheirality_)
        throw e;
      return camera_.defaultErrorWhenTriangulatingBehindCamera();
    }
  }

  /// thread-safe (?) scratch memory for linearize
  mutable VerticalBlockMatrix Ab;
  mutable Matrix A;
  mutable Vector b;

  /**
   * Linearize to a JacobianFactor, does not support constrained noise model !
   * \f$ Ax-b \approx h(x+\delta x)-z = h(x) + A \delta x - z \f$
   * Hence \f$ b = z - h(x) = - \mathtt{error\_vector}(x) \f$
   */
  std::shared_ptr<GaussianFactor> linearize(const Values& x) const override {
    // Only linearize if the factor is active
    if (!this->active(x))
      return std::shared_ptr<JacobianFactor>();

    // Allocate memory for Jacobian factor, do only once
    if (Ab.rows() == 0) {
      std::vector<size_t> dimensions(1, 3);
      Ab = VerticalBlockMatrix(dimensions, traits<Measurement>::dimension, true);
      A.resize(traits<Measurement>::dimension,3);
      b.resize(traits<Measurement>::dimension);
    }

    // Would be even better if we could pass blocks to project
    const Point3& point = x.at<Point3>(key());
    b = traits<Measurement>::Local(camera_.project2(point, {}, A), measured_);
    if (noiseModel_)
      this->noiseModel_->WhitenSystem(A, b);

    Ab(0) = A;
    Ab(1) = b;

    return std::make_shared<JacobianFactor>(this->keys_, Ab);
  }

  /** return the measurement */
  const Measurement& measured() const {
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

#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION  ///
  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar & BOOST_SERIALIZATION_NVP(camera_);
    ar & BOOST_SERIALIZATION_NVP(measured_);
    ar & BOOST_SERIALIZATION_NVP(throwCheirality_);
    ar & BOOST_SERIALIZATION_NVP(verboseCheirality_);
  }
#endif
};
} // \ namespace gtsam

