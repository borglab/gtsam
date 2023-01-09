/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file ProjectionFactor.h
 * @brief Reprojection of a LANDMARK to a 2D point.
 * @author Chris Beall
 * @author Richard Roberts
 * @author Frank Dellaert
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <boost/optional.hpp>

namespace gtsam {

  /**
   * Non-linear factor for a constraint derived from a 2D measurement. 
   * The calibration is known here.
   * The main building block for visual SLAM.
   * @ingroup slam
   */
  template <class POSE = Pose3, class LANDMARK = Point3,
            class CALIBRATION = Cal3_S2>
  class GenericProjectionFactor: public NoiseModelFactorN<POSE, LANDMARK> {
  protected:

    // Keep a copy of measurement and calibration for I/O
    Point2 measured_;                    ///< 2D measurement
    boost::shared_ptr<CALIBRATION> K_;  ///< shared pointer to calibration object
    boost::optional<POSE> body_P_sensor_; ///< The pose of the sensor in the body frame

    // verbosity handling for Cheirality Exceptions
    bool throwCheirality_; ///< If true, rethrows Cheirality exceptions (default: false)
    bool verboseCheirality_; ///< If true, prints text for Cheirality exceptions (default: false)

  public:

    /// shorthand for base class type
    typedef NoiseModelFactor2<POSE, LANDMARK> Base;
    using Base::evaluateError;

    /// shorthand for this class
    typedef GenericProjectionFactor<POSE, LANDMARK, CALIBRATION> This;

    /// shorthand for a smart pointer to a factor
    typedef boost::shared_ptr<This> shared_ptr;

    /// Default constructor
    GenericProjectionFactor() :
        measured_(0, 0), throwCheirality_(false), verboseCheirality_(false) {
    }

    /**
     * Constructor
     * TODO: Mark argument order standard (keys, measurement, parameters)
     * @param measured is the 2 dimensional location of point in image (the measurement)
     * @param model is the standard deviation
     * @param poseKey is the index of the camera
     * @param pointKey is the index of the landmark
     * @param K shared pointer to the constant calibration
     * @param body_P_sensor is the transform from body to sensor frame (default identity)
     */
    GenericProjectionFactor(const Point2& measured, const SharedNoiseModel& model,
        Key poseKey, Key pointKey, const boost::shared_ptr<CALIBRATION>& K,
        boost::optional<POSE> body_P_sensor = boost::none) :
          Base(model, poseKey, pointKey), measured_(measured), K_(K), body_P_sensor_(body_P_sensor),
          throwCheirality_(false), verboseCheirality_(false) {}

    /**
     * Constructor with exception-handling flags
     * TODO: Mark argument order standard (keys, measurement, parameters)
     * @param measured is the 2 dimensional location of point in image (the measurement)
     * @param model is the standard deviation
     * @param poseKey is the index of the camera
     * @param pointKey is the index of the landmark
     * @param K shared pointer to the constant calibration
     * @param throwCheirality determines whether Cheirality exceptions are rethrown
     * @param verboseCheirality determines whether exceptions are printed for Cheirality
     * @param body_P_sensor is the transform from body to sensor frame  (default identity)
     */
    GenericProjectionFactor(const Point2& measured, const SharedNoiseModel& model,
        Key poseKey, Key pointKey, const boost::shared_ptr<CALIBRATION>& K,
        bool throwCheirality, bool verboseCheirality,
        boost::optional<POSE> body_P_sensor = boost::none) :
          Base(model, poseKey, pointKey), measured_(measured), K_(K), body_P_sensor_(body_P_sensor),
          throwCheirality_(throwCheirality), verboseCheirality_(verboseCheirality) {}

    /** Virtual destructor */
    ~GenericProjectionFactor() override {}

    /// @return a deep copy of this factor
    gtsam::NonlinearFactor::shared_ptr clone() const override {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

    /**
     * print
     * @param s optional string naming the factor
     * @param keyFormatter optional formatter useful for printing Symbols
     */
    void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
      std::cout << s << "GenericProjectionFactor, z = ";
      traits<Point2>::Print(measured_);
      if(this->body_P_sensor_)
        this->body_P_sensor_->print("  sensor pose in body frame: ");
      Base::print("", keyFormatter);
    }

    /// equals
    bool equals(const NonlinearFactor& p, double tol = 1e-9) const override {
      const This *e = dynamic_cast<const This*>(&p);
      return e
          && Base::equals(p, tol)
          && traits<Point2>::Equals(this->measured_, e->measured_, tol)
          && this->K_->equals(*e->K_, tol)
          && ((!body_P_sensor_ && !e->body_P_sensor_) || (body_P_sensor_ && e->body_P_sensor_ && body_P_sensor_->equals(*e->body_P_sensor_)));
    }

    /// Evaluate error h(x)-z and optionally derivatives
    Vector evaluateError(const Pose3& pose, const Point3& point,
        OptionalMatrixType H1, OptionalMatrixType H2) const override {
      try {
        if(body_P_sensor_) {
          if(H1) {
            gtsam::Matrix H0;
            PinholeCamera<CALIBRATION> camera(pose.compose(*body_P_sensor_, H0), *K_);
            Point2 reprojectionError(camera.project(point, H1, H2, boost::none) - measured_);
            *H1 = *H1 * H0;
            return reprojectionError;
          } else {
            PinholeCamera<CALIBRATION> camera(pose.compose(*body_P_sensor_), *K_);
            return camera.project(point, H1, H2, boost::none) - measured_;
          }
        } else {
          PinholeCamera<CALIBRATION> camera(pose, *K_);
          return camera.project(point, H1, H2, boost::none) - measured_;
        }
      } catch( CheiralityException& e) {
        if (H1) *H1 = Matrix::Zero(2,6);
        if (H2) *H2 = Matrix::Zero(2,3);
        if (verboseCheirality_)
          std::cout << e.what() << ": Landmark "<< DefaultKeyFormatter(this->key2()) <<
              " moved behind camera " << DefaultKeyFormatter(this->key1()) << std::endl;
        if (throwCheirality_)
          throw CheiralityException(this->key2());
      }
      return Vector2::Constant(2.0 * K_->fx());
    }

    /** return the measurement */
    const Point2& measured() const {
      return measured_;
    }

    /** return the calibration object */
    const boost::shared_ptr<CALIBRATION> calibration() const {
      return K_;
    }

    /** return the (optional) sensor pose with respect to the vehicle frame */
    const boost::optional<POSE>& body_P_sensor() const {
      return body_P_sensor_;
    }

    /** return verbosity */
    inline bool verboseCheirality() const { return verboseCheirality_; }

    /** return flag for throwing cheirality exceptions */
    inline bool throwCheirality() const { return throwCheirality_; }

  private:

    /// Serialization function
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
      ar & BOOST_SERIALIZATION_NVP(measured_);
      ar & BOOST_SERIALIZATION_NVP(K_);
      ar & BOOST_SERIALIZATION_NVP(body_P_sensor_);
      ar & BOOST_SERIALIZATION_NVP(throwCheirality_);
      ar & BOOST_SERIALIZATION_NVP(verboseCheirality_);
    }

  public:
    GTSAM_MAKE_ALIGNED_OPERATOR_NEW
};

  /// traits
  template<class POSE, class LANDMARK, class CALIBRATION>
  struct traits<GenericProjectionFactor<POSE, LANDMARK, CALIBRATION> > :
      public Testable<GenericProjectionFactor<POSE, LANDMARK, CALIBRATION> > {
  };

} // \ namespace gtsam
