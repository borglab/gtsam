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

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/CalibratedCamera.h>
#include <boost/make_shared.hpp>
#include <boost/lexical_cast.hpp>

namespace gtsam {

/**
 * Non-linear factor for a constraint derived from a 2D measurement.
 * The calibration and pose are assumed known.
 * @addtogroup SLAM
 */
template<class CAMERA>
class TriangulationFactor: public NoiseModelFactor1<Point3> {

public:

  /// CAMERA type
  typedef CAMERA Camera;

protected:

  /// shorthand for base class type
  typedef NoiseModelFactor1<Point3> Base;

  /// shorthand for this class
  typedef TriangulationFactor<CAMERA> This;

  /// shorthand for measurement type, e.g. Point2 or StereoPoint2
  typedef typename CAMERA::Measurement Measurement;

  // Keep a copy of measurement and calibration for I/O
  const CAMERA camera_; ///< CAMERA in which this landmark was seen
  const Measurement measured_; ///< 2D measurement

  // verbosity handling for Cheirality Exceptions
  const bool throwCheirality_; ///< If true, rethrows Cheirality exceptions (default: false)
  const bool verboseCheirality_; ///< If true, prints text for Cheirality exceptions (default: false)

public:

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
  TriangulationFactor(const CAMERA& camera, const Measurement& measured,
      const SharedNoiseModel& model, Key pointKey, bool throwCheirality = false,
      bool verboseCheirality = false) :
      Base(model, pointKey), camera_(camera), measured_(measured), throwCheirality_(
          throwCheirality), verboseCheirality_(verboseCheirality) {
    if (model && model->dim() != traits<Measurement>::dimension)
      throw std::invalid_argument(
          "TriangulationFactor must be created with "
              + boost::lexical_cast<std::string>((int) traits<Measurement>::dimension)
              + "-dimensional noise model.");
  }

  /** Virtual destructor */
  ~TriangulationFactor() override {
  }

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
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
  Vector evaluateError(const Point3& point, boost::optional<Matrix&> H2 =
      boost::none) const override {
    try {
      return traits<Measurement>::Local(measured_, camera_.project2(point, boost::none, H2));
    } catch (CheiralityException& e) {
      if (H2)
        *H2 = Matrix::Zero(traits<Measurement>::dimension, 3);
      if (verboseCheirality_)
        std::cout << e.what() << ": Landmark "
            << DefaultKeyFormatter(this->key()) << " moved behind camera"
            << std::endl;
      if (throwCheirality_)
        throw e;
      return Eigen::Matrix<double,traits<Measurement>::dimension,1>::Constant(2.0 * camera_.calibration().fx());
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
  boost::shared_ptr<GaussianFactor> linearize(const Values& x) const override {
    // Only linearize if the factor is active
    if (!this->active(x))
      return boost::shared_ptr<JacobianFactor>();

    // Allocate memory for Jacobian factor, do only once
    if (Ab.rows() == 0) {
      std::vector<size_t> dimensions(1, 3);
      Ab = VerticalBlockMatrix(dimensions, traits<Measurement>::dimension, true);
      A.resize(traits<Measurement>::dimension,3);
      b.resize(traits<Measurement>::dimension);
    }

    // Would be even better if we could pass blocks to project
    const Point3& point = x.at<Point3>(key());
    b = traits<Measurement>::Local(camera_.project2(point, boost::none, A), measured_);
    if (noiseModel_)
      this->noiseModel_->WhitenSystem(A, b);

    Ab(0) = A;
    Ab(1) = b;

    return boost::make_shared<JacobianFactor>(this->keys_, Ab);
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
};
} // \ namespace gtsam

