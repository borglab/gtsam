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
#include <gtsam/geometry/Pose3.h>
#include <gtsam_unstable/geometry/triangulation.h>
#include <boost/optional.hpp>
#include <boost/assign.hpp>

namespace gtsam {

  /**
   * Non-linear factor for a constraint derived from a 2D measurement. The calibration is known here.
   * i.e. the main building block for visual SLAM.
   * @addtogroup SLAM
   */
  template<class POSE, class LANDMARK, class CALIBRATION = Cal3_S2>
  class SmartProjectionFactor: public NonlinearFactor {
  protected:

    // Keep a copy of measurement and calibration for I/O
    std::vector<Point2> measured_;                    ///< 2D measurement for each of the n views
    ///< (important that the order is the same as the keys that we use to create the factor)
    boost::shared_ptr<CALIBRATION> K_;  ///< shared pointer to calibration object
    const SharedNoiseModel noise_;   ///< noise model used
    boost::optional<POSE> body_P_sensor_; ///< The pose of the sensor in the body frame



    // verbosity handling for Cheirality Exceptions
    bool throwCheirality_; ///< If true, rethrows Cheirality exceptions (default: false)
    bool verboseCheirality_; ///< If true, prints text for Cheirality exceptions (default: false)

  public:

    /// shorthand for base class type
    typedef NonlinearFactor Base;

    /// shorthand for this class
    typedef SmartProjectionFactor<POSE, LANDMARK, CALIBRATION> This;

    /// shorthand for a smart pointer to a factor
    typedef boost::shared_ptr<This> shared_ptr;

    /// Default constructor
    SmartProjectionFactor() : throwCheirality_(false), verboseCheirality_(false) {}

    /**
     * Constructor
     * TODO: Mark argument order standard (keys, measurement, parameters)
     * @param measured is the 2n dimensional location of the n points in the n views (the measurements)
     * @param model is the standard deviation (current version assumes that the uncertainty is the same for all views)
     * @param poseKeys is the set of indices corresponding to the cameras observing the same landmark
     * @param K shared pointer to the constant calibration
     * @param body_P_sensor is the transform from body to sensor frame (default identity)
     */
    SmartProjectionFactor(const std::vector<Point2> measured, const SharedNoiseModel& model,
        std::vector<Key> poseKeys, const boost::shared_ptr<CALIBRATION>& K,
        boost::optional<POSE> body_P_sensor = boost::none) :
          measured_(measured),  K_(K), noise_(model), body_P_sensor_(body_P_sensor),
          throwCheirality_(false), verboseCheirality_(false) {
      keys_.assign(poseKeys.begin(), poseKeys.end());
    }

    /**
     * Constructor with exception-handling flags
     * TODO: Mark argument order standard (keys, measurement, parameters)
     * @param measured is the 2 dimensional location of point in image (the measurement)
     * @param model is the standard deviation
     * @param poseKey is the index of the camera
     * @param K shared pointer to the constant calibration
     * @param throwCheirality determines whether Cheirality exceptions are rethrown
     * @param verboseCheirality determines whether exceptions are printed for Cheirality
     * @param body_P_sensor is the transform from body to sensor frame  (default identity)
     */
    SmartProjectionFactor(const std::vector<Point2> measured, const SharedNoiseModel& model,
        std::vector<Key> poseKeys, const boost::shared_ptr<CALIBRATION>& K,
        bool throwCheirality, bool verboseCheirality,
        boost::optional<POSE> body_P_sensor = boost::none) :
          measured_(measured),  K_(K),  noise_(model), body_P_sensor_(body_P_sensor),
          throwCheirality_(throwCheirality), verboseCheirality_(verboseCheirality) {}

    /** Virtual destructor */
    virtual ~SmartProjectionFactor() {}

    /// @return a deep copy of this factor
//    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
//      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
//          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

    /**
     * print
     * @param s optional string naming the factor
     * @param keyFormatter optional formatter useful for printing Symbols
     */
    void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
      std::cout << s << "SmartProjectionFactor, z = ";
      BOOST_FOREACH(const Point2& p, measured_) {
        std::cout << "measurement, p = "<< p << std::endl;
      }
      if(this->body_P_sensor_)
        this->body_P_sensor_->print("  sensor pose in body frame: ");
      Base::print("", keyFormatter);
    }

    /// equals
    virtual bool equals(const NonlinearFactor& p, double tol = 1e-9) const {
      const This *e = dynamic_cast<const This*>(&p);

      bool areMeasurementsEqual = true;
      for(size_t i = 0; i < measured_.size(); i++) {
        if(this->measured_.at(i).equals(e->measured_.at(i), tol) == false)
          areMeasurementsEqual = false;
        break;
      }

      return e
          && Base::equals(p, tol)
          && areMeasurementsEqual
          && this->K_->equals(*e->K_, tol)
          && ((!body_P_sensor_ && !e->body_P_sensor_) || (body_P_sensor_ && e->body_P_sensor_ && body_P_sensor_->equals(*e->body_P_sensor_)));
    }

    /// Evaluate error h(x)-z and optionally derivatives
    Vector unwhitenedError(const Values& x, boost::optional<std::vector<Matrix>&> H = boost::none) const{

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
//            gtsam::Matrix H0;
//            PinholeCamera<CALIBRATION> camera(pose.compose(*body_P_sensor_, H0), *K_);
//            Point2 reprojectionError(camera.project(point, H1, H2) - measured_);
//            *H1 = *H1 * H0;
//            return reprojectionError.vector();
//          } else {
//            PinholeCamera<CALIBRATION> camera(pose.compose(*body_P_sensor_), *K_);
//            Point2 reprojectionError(camera.project(point, H1, H2) - measured_);
//            return reprojectionError.vector();
//          }
//        } else {
//          PinholeCamera<CALIBRATION> camera(pose, *K_);
//          Point2 reprojectionError(camera.project(point, H1, H2) - measured_);
//          return reprojectionError.vector();
//        }
//      }

    }

    /// get the dimension of the factor (number of rows on linearization)
    virtual size_t dim() const {
        return 6*keys_.size();
    }

    /// linearize returns a Hessianfactor that is an approximation of error(p)
    virtual boost::shared_ptr<GaussianFactor> linearize(const Values& x,  const Ordering& ordering) const {

      // fill in the keys
      std::vector<Index> js;
      BOOST_FOREACH(const Key& k, keys_) {
        js += ordering[k];
      }


      std::vector<Matrix> Gs;

      std::vector<Vector> gs;
      // Shur complement trick

//      double e = u + b - z , e2 = e * e;
//      double c = 2 * logSqrt2PI - log(p) + e2 * p;
//      Vector g1 = Vector_(1, -e * p);
//      Vector g2 = Vector_(1, 0.5 / p - 0.5 * e2);
//      Vector g3 = Vector_(1, -e * p);
//      Matrix G11 = Matrix_(1, 1, p);
//      Matrix G12 = Matrix_(1, 1, e);
//      Matrix G13 = Matrix_(1, 1, p);
//      Matrix G22 = Matrix_(1, 1, 0.5 / (p * p));
//      Matrix G23 = Matrix_(1, 1, e);
//      Matrix G33 = Matrix_(1, 1, p);

      double f = 0;

      return HessianFactor::shared_ptr(new HessianFactor(js, Gs, gs, f));
    }

    /**
     * Calculate the error of the factor.
     * This is the log-likelihood, e.g. \f$ 0.5(h(x)-z)^2/\sigma^2 \f$ in case of Gaussian.
     * In this class, we take the raw prediction error \f$ h(x)-z \f$, ask the noise model
     * to transform it to \f$ (h(x)-z)^2/\sigma^2 \f$, and then multiply by 0.5.
     */
    virtual double error(const Values& values) const {
      if (this->active(values)) {
        double overallError=0;

        // Collect all poses (Cameras)
        std::vector<Pose3> cameraPoses;

        BOOST_FOREACH(const Key& k, keys_) {
          if(body_P_sensor_)
            cameraPoses.push_back(values.at<Pose3>(k).compose(*body_P_sensor_));
          else
            cameraPoses.push_back(values.at<Pose3>(k));
        }

        // We triangulate the 3D position of the landmark
        boost::optional<Point3> point = triangulatePoint3(cameraPoses, measured_, *K_);

        if(point)
        { // triangulation produced a good estimate of landmark position

          std::cout << "point " << *point << std::endl;

          for(size_t i = 0; i < measured_.size(); i++) {
            Pose3 pose = cameraPoses.at(i);
            PinholeCamera<CALIBRATION> camera(pose, *K_);
            std::cout << "pose.compose(*body_P_sensor_) " << pose << std::endl;

            Point2 reprojectionError(camera.project(*point) - measured_.at(i));
            std::cout << "reprojectionError " << reprojectionError << std::endl;
            overallError += noise_->distance( reprojectionError.vector() );
            std::cout << "noise_->distance( reprojectionError.vector() ) " << noise_->distance( reprojectionError.vector() ) << std::endl;
          }
          return sqrt(overallError);
        }else{ // triangulation failed: we deactivate the factor, then the error should not contribute to the overall error
          return 0.0;
        }
      } else {
        return 0.0;
      }
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
    void serialize(ARCHIVE & ar, const unsigned int version) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
      ar & BOOST_SERIALIZATION_NVP(measured_);
      ar & BOOST_SERIALIZATION_NVP(K_);
      ar & BOOST_SERIALIZATION_NVP(body_P_sensor_);
      ar & BOOST_SERIALIZATION_NVP(throwCheirality_);
      ar & BOOST_SERIALIZATION_NVP(verboseCheirality_);
    }
  };
} // \ namespace gtsam
