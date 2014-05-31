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

#include "SmartProjectionFactor.h"

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
template<class POSE, class LANDMARK, class CALIBRATION>
class SmartProjectionPoseFactor: public SmartProjectionFactor<POSE, LANDMARK, CALIBRATION, 6> {
protected:

  LinearizationMode linearizeTo_;  ///< How to linearize the factor (HESSIAN, JACOBIAN_SVD, JACOBIAN_Q)

  std::vector<boost::shared_ptr<CALIBRATION> > K_all_; ///< shared pointer to calibration object (one for each camera)

public:

  /// shorthand for base class type
  typedef SmartProjectionFactor<POSE, LANDMARK, CALIBRATION, 6> Base;

  /// shorthand for this class
  typedef SmartProjectionPoseFactor<POSE, LANDMARK, CALIBRATION> This;

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /**
   * Constructor
   * @param rankTol tolerance used to check if point triangulation is degenerate
   * @param linThreshold threshold on relative pose changes used to decide whether to relinearize (selective relinearization)
   * @param manageDegeneracy is true, in presence of degenerate triangulation, the factor is converted to a rotation-only constraint,
   * otherwise the factor is simply neglected
   * @param enableEPI if set to true linear triangulation is refined with embedded LM iterations
   * @param body_P_sensor is the transform from body to sensor frame (default identity)
   */
  SmartProjectionPoseFactor(const double rankTol = 1,
      const double linThreshold = -1, const bool manageDegeneracy = false,
      const bool enableEPI = false, boost::optional<POSE> body_P_sensor = boost::none,
      LinearizationMode linearizeTo = HESSIAN, double landmarkDistanceThreshold = 1e10,
      double dynamicOutlierRejectionThreshold = -1) :
        Base(rankTol, linThreshold, manageDegeneracy, enableEPI, body_P_sensor,
        landmarkDistanceThreshold, dynamicOutlierRejectionThreshold), linearizeTo_(linearizeTo) {}

  /** Virtual destructor */
  virtual ~SmartProjectionPoseFactor() {}

  /**
   * add a new measurement and pose key
   * @param measured is the 2m dimensional location of the projection of a single landmark in the m view (the measurement)
   * @param poseKey is key corresponding to the camera observing the same landmark
   * @param noise_i is the measurement noise
   * @param K_i is the (known) camera calibration
   */
  void add(const Point2 measured_i, const Key poseKey_i,
      const SharedNoiseModel noise_i,
      const boost::shared_ptr<CALIBRATION> K_i) {
    Base::add(measured_i, poseKey_i, noise_i);
    K_all_.push_back(K_i);
  }

  /**
   *  Variant of the previous one in which we include a set of measurements
   * @param measurements vector of the 2m dimensional location of the projection of a single landmark in the m view (the measurement)
   * @param poseKeys vector of keys corresponding to the camera observing the same landmark
   * @param noises vector of measurement noises
   * @param Ks vector of calibration objects
   */
  void add(std::vector<Point2> measurements, std::vector<Key> poseKeys,
      std::vector<SharedNoiseModel> noises,
      std::vector<boost::shared_ptr<CALIBRATION> > Ks) {
    Base::add(measurements, poseKeys, noises);
    for (size_t i = 0; i < measurements.size(); i++) {
      K_all_.push_back(Ks.at(i));
    }
  }

  /**
   * Variant of the previous one in which we include a set of measurements with the same noise and calibration
   * @param mmeasurements vector of the 2m dimensional location of the projection of a single landmark in the m view (the measurement)
   * @param poseKeys vector of keys corresponding to the camera observing the same landmark
   * @param noise measurement noise (same for all measurements)
   * @param K the (known) camera calibration (same for all measurements)
   */
  void add(std::vector<Point2> measurements, std::vector<Key> poseKeys,
      const SharedNoiseModel noise, const boost::shared_ptr<CALIBRATION> K) {
    for (size_t i = 0; i < measurements.size(); i++) {
      Base::add(measurements.at(i), poseKeys.at(i), noise);
      K_all_.push_back(K);
    }
  }

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
      DefaultKeyFormatter) const {
    std::cout << s << "SmartProjectionPoseFactor, z = \n ";
    BOOST_FOREACH(const boost::shared_ptr<CALIBRATION>& K, K_all_)
    K->print("calibration = ");
    Base::print("", keyFormatter);
  }

  /// equals
  virtual bool equals(const NonlinearFactor& p, double tol = 1e-9) const {
    const This *e = dynamic_cast<const This*>(&p);

    return e && Base::equals(p, tol);
  }

  /// get the dimension of the factor
  virtual size_t dim() const {
    return 6 * this->keys_.size();
  }

  /**
   * Collect all cameras involved in this factor
   * @param values Values structure which must contain camera poses corresponding
   * to keys involved in this factor
   * @return vector of Values
   */
  typename Base::Cameras cameras(const Values& values) const {
    typename Base::Cameras cameras;
    size_t i=0;
    BOOST_FOREACH(const Key& k, this->keys_) {
      Pose3 pose = values.at<Pose3>(k);
      typename Base::Camera camera(pose, *K_all_[i++]);
      cameras.push_back(camera);
    }
    return cameras;
  }

  /**
   * Linearize to Gaussian Factor
   * @param values Values structure which must contain camera poses for this factor
   * @return
   */
  virtual boost::shared_ptr<GaussianFactor> linearize(
      const Values& values) const {
    // depending on flag set on construction we may linearize to different linear factors
    switch(linearizeTo_){
    case JACOBIAN_SVD :
      return this->createJacobianSVDFactor(cameras(values), 0.0);
      break;
    case JACOBIAN_Q :
      return this->createJacobianQFactor(cameras(values), 0.0);
      break;
    default:
      return this->createHessianFactor(cameras(values));
      break;
    }
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

  /** return the calibration object */
  inline const std::vector<boost::shared_ptr<CALIBRATION> > calibration() const {
    return K_all_;
  }

private:

  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar & BOOST_SERIALIZATION_NVP(K_all_);
  }

}; // end of class declaration

} // \ namespace gtsam
