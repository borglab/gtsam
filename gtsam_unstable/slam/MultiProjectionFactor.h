/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file ProjectionFactor.h
 * @brief Basic bearing factor from 2D measurement
 * @author Chris Beall
 * @author Richard Roberts
 * @author Frank Dellaert
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <boost/optional.hpp>

namespace gtsam {

  /**
   * Non-linear factor for a constraint derived from a 2D measurement. The calibration is known here.
   * i.e. the main building block for visual SLAM.
   * @ingroup slam
   */
  template<class POSE, class LANDMARK, class CALIBRATION = Cal3_S2>
  class MultiProjectionFactor: public NoiseModelFactor {
  protected:

    // Keep a copy of measurement and calibration for I/O
    Vector measured_;                    ///< 2D measurement for each of the n views
    boost::shared_ptr<CALIBRATION> K_;  ///< shared pointer to calibration object
    boost::optional<POSE> body_P_sensor_; ///< The pose of the sensor in the body frame


    // verbosity handling for Cheirality Exceptions
    bool throwCheirality_; ///< If true, rethrows Cheirality exceptions (default: false)
    bool verboseCheirality_; ///< If true, prints text for Cheirality exceptions (default: false)

  public:

    /// shorthand for base class type
    typedef NoiseModelFactor Base;

    /// shorthand for this class
    typedef MultiProjectionFactor<POSE, LANDMARK, CALIBRATION> This;

    /// shorthand for a smart pointer to a factor
    typedef boost::shared_ptr<This> shared_ptr;

    /// Default constructor
    MultiProjectionFactor() : throwCheirality_(false), verboseCheirality_(false) {}

    /**
     * Constructor
     * TODO: Mark argument order standard (keys, measurement, parameters)
     * @param measured is the 2n dimensional location of the n points in the n views (the measurements)
     * @param model is the standard deviation (current version assumes that the uncertainty is the same for all views)
     * @param poseKeys is the set of indices corresponding to the cameras observing the same landmark
     * @param pointKey is the index of the landmark
     * @param K shared pointer to the constant calibration
     * @param body_P_sensor is the transform from body to sensor frame (default identity)
     */
    MultiProjectionFactor(const Vector& measured, const SharedNoiseModel& model,
        KeySet poseKeys, Key pointKey, const boost::shared_ptr<CALIBRATION>& K,
        boost::optional<POSE> body_P_sensor = boost::none) :
          Base(model), measured_(measured), K_(K), body_P_sensor_(body_P_sensor),
          throwCheirality_(false), verboseCheirality_(false) {
      keys_.assign(poseKeys.begin(), poseKeys.end());
      keys_.push_back(pointKey);
    }

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
    MultiProjectionFactor(const Vector& measured, const SharedNoiseModel& model,
        KeySet poseKeys, Key pointKey, const boost::shared_ptr<CALIBRATION>& K,
        bool throwCheirality, bool verboseCheirality,
        boost::optional<POSE> body_P_sensor = boost::none) :
          Base(model), measured_(measured), K_(K), body_P_sensor_(body_P_sensor),
          throwCheirality_(throwCheirality), verboseCheirality_(verboseCheirality) {}

    /** Virtual destructor */
    ~MultiProjectionFactor() override {}

    /// @return a deep copy of this factor
    NonlinearFactor::shared_ptr clone() const override {
      return boost::static_pointer_cast<NonlinearFactor>(
          NonlinearFactor::shared_ptr(new This(*this))); }

    /**
     * print
     * @param s optional string naming the factor
     * @param keyFormatter optional formatter useful for printing Symbols
     */
    void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
      std::cout << s << "MultiProjectionFactor, z = ";
      std::cout << measured_ << "measurements, z = ";
      if(this->body_P_sensor_)
        this->body_P_sensor_->print("  sensor pose in body frame: ");
      Base::print("", keyFormatter);
    }

    /// equals
    bool equals(const NonlinearFactor& p, double tol = 1e-9) const override {
      const This *e = dynamic_cast<const This*>(&p);
      return e
          && Base::equals(p, tol)
          //&& this->measured_.equals(e->measured_, tol)
          && this->K_->equals(*e->K_, tol)
          && ((!body_P_sensor_ && !e->body_P_sensor_) || (body_P_sensor_ && e->body_P_sensor_ && body_P_sensor_->equals(*e->body_P_sensor_)));
    }

    /// Evaluate error h(x)-z and optionally derivatives
    Vector unwhitenedError(const Values& x, OptionalMatrixVecType H = OptionalNone) const override {

      Vector a;
      return a;

//      Point3 point = x.at<Point3>(*keys_.end());
//
//      std::vector<KeyType>::iterator vit;
//      for (vit = keys_.begin(); vit != keys_.end()-1; vit++) {
//        Key key = (*vit);
//        Pose3 pose = x.at<Pose3>(key);
//
//        if(body_P_sensor_) {
//          if(H1) {
//            Matrix H0;
//            PinholeCamera<CALIBRATION> camera(pose.compose(*body_P_sensor_, H0), *K_);
//            Point2 reprojectionError(camera.project(point, H1, H2) - measured_);
//            *H1 = *H1 * H0;
//            return reprojectionError;
//          } else {
//            PinholeCamera<CALIBRATION> camera(pose.compose(*body_P_sensor_), *K_);
//            Point2 reprojectionError(camera.project(point, H1, H2) - measured_);
//            return reprojectionError;
//          }
//        } else {
//          PinholeCamera<CALIBRATION> camera(pose, *K_);
//          Point2 reprojectionError(camera.project(point, H1, H2) - measured_);
//          return reprojectionError;
//        }
//      }

    }


    Vector evaluateError(const Pose3& pose, const Point3& point,
        boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none) const {
      try {
        if(body_P_sensor_) {
          if(H1) {
            Matrix H0;
            PinholeCamera<CALIBRATION> camera(pose.compose(*body_P_sensor_, H0), *K_);
            Point2 reprojectionError(camera.project(point, H1, H2) - measured_);
            *H1 = *H1 * H0;
            return reprojectionError;
          } else {
            PinholeCamera<CALIBRATION> camera(pose.compose(*body_P_sensor_), *K_);
            Point2 reprojectionError(camera.project(point, H1, H2) - measured_);
            return reprojectionError;
          }
        } else {
          PinholeCamera<CALIBRATION> camera(pose, *K_);
          Point2 reprojectionError(camera.project(point, H1, H2) - measured_);
          return reprojectionError;
        }
      } catch( CheiralityException& e) {
        if (H1) *H1 = Matrix::Zero(2,6);
        if (H2) *H2 = Matrix::Zero(2,3);
        if (verboseCheirality_)
          std::cout << e.what() << ": Landmark "<< DefaultKeyFormatter(this->keys_.at(1)) <<
              " moved behind camera " << DefaultKeyFormatter(this->keys_.at(0)) << std::endl;
        if (throwCheirality_)
          throw e;
      }
      return Vector::Ones(2) * 2.0 * K_->fx();
    }

    /** return the measurements */
    const Vector& measured() const {
      return measured_;
    }

    /** return the calibration object */
    inline const boost::shared_ptr<CALIBRATION> calibration() const {
      return K_;
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
  };
} // \ namespace gtsam
