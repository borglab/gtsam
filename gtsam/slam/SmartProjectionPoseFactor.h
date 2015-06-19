/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   SmartProjectionPoseFactor.h
 * @brief  Produces an Hessian factors on POSES from monocular measurements of a single landmark
 * @author Luca Carlone
 * @author Chris Beall
 * @author Zsolt Kira
 */

#pragma once

#include <gtsam/slam/SmartProjectionFactor.h>

namespace gtsam {
/**
 *
 * @addtogroup SLAM
 *
 * If you are using the factor, please cite:
 * L. Carlone, Z. Kira, C. Beall, V. Indelman, F. Dellaert, Eliminating conditionally
 * independent sets in factor graphs: a unifying perspective based on smart factors,
 * Int. Conf. on Robotics and Automation (ICRA), 2014.
 *
 */

/**
 * The calibration is known here. The factor only constraints poses (variable dimension is 6)
 * @addtogroup SLAM
 */
template<class CALIBRATION>
class SmartProjectionPoseFactor: public SmartProjectionFactor<
    PinholePose<CALIBRATION> > {

private:
  typedef PinholePose<CALIBRATION> Camera;
  typedef SmartProjectionFactor<Camera> Base;
  typedef SmartProjectionPoseFactor<CALIBRATION> This;

protected:

  boost::shared_ptr<CALIBRATION> K_; ///< calibration object (one for all cameras)

public:

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /**
   * Constructor
   * @param rankTol tolerance used to check if point triangulation is degenerate
   * @param linThreshold threshold on relative pose changes used to decide whether to relinearize (selective relinearization)
   * @param manageDegeneracy is true, in presence of degenerate triangulation, the factor is converted to a rotation-only constraint,
   * otherwise the factor is simply neglected (this functionality is deprecated)
   * @param enableEPI if set to true linear triangulation is refined with embedded LM iterations
   * @param body_P_sensor is the transform from sensor to body frame (default identity)
   */
  SmartProjectionPoseFactor(const boost::shared_ptr<CALIBRATION> K, const double rankTol = 1,
      const double linThreshold = -1, const DegeneracyMode manageDegeneracy = IGNORE_DEGENERACY,
      const bool enableEPI = false, boost::optional<Pose3> body_P_sensor = boost::none,
      LinearizationMode linearizeTo = HESSIAN, double landmarkDistanceThreshold = 1e10,
      double dynamicOutlierRejectionThreshold = -1) :
        Base(linearizeTo, rankTol, manageDegeneracy, enableEPI, landmarkDistanceThreshold,
            dynamicOutlierRejectionThreshold, body_P_sensor), K_(K) {}

  /** Virtual destructor */
  virtual ~SmartProjectionPoseFactor() {}

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
      DefaultKeyFormatter) const {
    std::cout << s << "SmartProjectionPoseFactor, z = \n ";
    Base::print("", keyFormatter);
  }

  /// equals
  virtual bool equals(const NonlinearFactor& p, double tol = 1e-9) const {
    const This *e = dynamic_cast<const This*>(&p);
    return e && Base::equals(p, tol);
  }

  /**
   * Linearize to Gaussian Factor
   * @param values Values structure which must contain camera poses for this factor
   * @return a Gaussian factor
   */
  boost::shared_ptr<GaussianFactor> linearizeDamped(const Values& values,
      const double lambda = 0.0) const {
    // depending on flag set on construction we may linearize to different linear factors
    typename Base::Cameras cameras = this->cameras(values);
    return Base::linearizeDamped(cameras, lambda);
  }

  /// linearize
  virtual boost::shared_ptr<GaussianFactor> linearize(
      const Values& values) const {
    return linearizeDamped(values);
  }

  /**
   * error calculates the error of the factor.
   */
  virtual double error(const Values& values) const {
    if (this->active(values)) {
      return this->totalReprojectionError(cameras(values));
    } else { // else of active flag
      return 0.0;
    }
  }

  /** return calibration shared pointers */
  inline const boost::shared_ptr<CALIBRATION> calibration() const {
    return K_;
  }

  /**
   * Collect all cameras involved in this factor
   * @param values Values structure which must contain camera poses corresponding
   * to keys involved in this factor
   * @return vector of Values
   */
  typename Base::Cameras cameras(const Values& values) const {
    typename Base::Cameras cameras;
    BOOST_FOREACH(const Key& k, this->keys_) {
      Pose3 pose = values.at<Pose3>(k);
      if(Base::body_P_sensor_)
        pose = pose.compose(*(Base::body_P_sensor_));

      Camera camera(pose, K_);
      cameras.push_back(camera);
    }
    return cameras;
  }

  /**
   * Triangulate and compute derivative of error with respect to point
   * @return whether triangulation worked
   */
  bool triangulateAndComputeE(Matrix& E, const Values& values) const {
    typename Base::Cameras cameras = this->cameras(values);
    return Base::triangulateAndComputeE(E, cameras);
  }

private:

  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar & BOOST_SERIALIZATION_NVP(K_);
  }

}; // end of class declaration

/// traits
template<class CALIBRATION>
struct traits<SmartProjectionPoseFactor<CALIBRATION> > : public Testable<
    SmartProjectionPoseFactor<CALIBRATION> > {
};

} // \ namespace gtsam
