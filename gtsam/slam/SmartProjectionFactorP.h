/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   SmartProjectionFactorP.h
 * @brief  Smart factor on poses, assuming camera calibration is fixed.
 *         Same as SmartProjectionPoseFactor, except:
 *         - it is templated on CAMERA (i.e., it allows cameras beyond pinhole)
 *         - it admits a different calibration for each measurement (i.e., it can model a multi-camera system)
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
 * This factor assumes that camera calibration is fixed (but each camera
 * measurement can have a different extrinsic and intrinsic calibration).
 * The factor only constrains poses (variable dimension is 6).
 * This factor requires that values contains the involved poses (Pose3).
 * If all measurements share the same calibration (i.e., are from the same camera), use SmartProjectionPoseFactor instead!
 * If the calibration should be optimized, as well, use SmartProjectionFactor instead!
 * @addtogroup SLAM
 */
template<class CAMERA>
class SmartProjectionFactorP : public SmartProjectionFactor<CAMERA> {

 private:
  typedef SmartProjectionFactor<CAMERA> Base;
  typedef SmartProjectionFactorP<CAMERA> This;
  typedef CAMERA Camera;
  typedef typename CAMERA::CalibrationType CALIBRATION;

 protected:

  /// shared pointer to calibration object (one for each observation)
  std::vector<boost::shared_ptr<CALIBRATION> > K_all_;

  /// Pose of the camera in the body frame (one for each observation)
  std::vector<Pose3> body_P_sensors_;

 public:

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /// Default constructor, only for serialization
  SmartProjectionFactorP() {
  }

  /**
   * Constructor
   * @param sharedNoiseModel isotropic noise model for the 2D feature measurements
   * @param params parameters for the smart projection factors
   */
  SmartProjectionFactorP(const SharedNoiseModel& sharedNoiseModel,
                         const SmartProjectionParams& params =
                             SmartProjectionParams())
      : Base(sharedNoiseModel, params) {
  }

  /** Virtual destructor */
  ~SmartProjectionFactorP() override {
  }

  /**
   * add a new measurement, corresponding to an observation from pose "poseKey" whose camera
   * has intrinsic calibration K and extrinsic calibration body_P_sensor.
   * @param measured 2-dimensional location of the projection of a
   * single landmark in a single view (the measurement)
   * @param poseKey key corresponding to the body pose of the camera taking the measurement
   * @param K (fixed) camera intrinsic calibration
   * @param body_P_sensor (fixed) camera extrinsic calibration
   */
  void add(const Point2& measured, const Key& poseKey,
           const boost::shared_ptr<CALIBRATION>& K, const Pose3 body_P_sensor =
               Pose3::identity()) {
    // store measurement and key
    this->measured_.push_back(measured);
    this->keys_.push_back(poseKey);
    // store fixed intrinsic calibration
    K_all_.push_back(K);
    // store fixed extrinsics of the camera
    body_P_sensors_.push_back(body_P_sensor);
  }

  /**
   * Variant of the previous "add" function in which we include multiple measurements
   * @param measurements vector of the 2m dimensional location of the projection
   * of a single landmark in the m views (the measurements)
   * @param poseKeys keys corresponding to the body poses of the cameras taking the measurements
   * @param Ks vector of (fixed) intrinsic calibration objects
   * @param body_P_sensors vector of (fixed) extrinsic calibration objects
   */
  void add(const Point2Vector& measurements, const std::vector<Key>& poseKeys,
           const std::vector<boost::shared_ptr<CALIBRATION>>& Ks,
           const std::vector<Pose3> body_P_sensors = std::vector<Pose3>()) {
    assert(poseKeys.size() == measurements.size());
    assert(poseKeys.size() == Ks.size());
    for (size_t i = 0; i < measurements.size(); i++) {
      if (poseKeys.size() == body_P_sensors.size()) {
        add(measurements[i], poseKeys[i], Ks[i], body_P_sensors[i]);
      } else {
        add(measurements[i], poseKeys[i], Ks[i]);  // use default extrinsics
      }
    }
  }

  /// return the calibration object
  inline std::vector<boost::shared_ptr<CALIBRATION>> calibration() const {
    return K_all_;
  }

  /// return the extrinsic camera calibration body_P_sensors
  const std::vector<Pose3> body_P_sensors() const {
    return body_P_sensors_;
  }

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                 DefaultKeyFormatter) const override {
    std::cout << s << "SmartProjectionFactorP: \n ";
    for (size_t i = 0; i < K_all_.size(); i++) {
      std::cout << "-- Measurement nr " << i << std::endl;
      body_P_sensors_[i].print("extrinsic calibration:\n");
      K_all_[i]->print("intrinsic calibration = ");
    }
    Base::print("", keyFormatter);
  }

  /// equals
  bool equals(const NonlinearFactor& p, double tol = 1e-9) const override {
    const This *e = dynamic_cast<const This*>(&p);
    double extrinsicCalibrationEqual = true;
    if (this->body_P_sensors_.size() == e->body_P_sensors().size()) {
      for (size_t i = 0; i < this->body_P_sensors_.size(); i++) {
        if (!body_P_sensors_[i].equals(e->body_P_sensors()[i])) {
          extrinsicCalibrationEqual = false;
          break;
        }
      }
    } else {
      extrinsicCalibrationEqual = false;
    }

    return e && Base::equals(p, tol) && K_all_ == e->calibration()
        && extrinsicCalibrationEqual;
  }

  /**
   * error calculates the error of the factor.
   */
  double error(const Values& values) const override {
    if (this->active(values)) {
      return this->totalReprojectionError(cameras(values));
    } else {  // else of active flag
      return 0.0;
    }
  }

  /**
   * Collect all cameras involved in this factor
   * @param values Values structure which must contain camera poses corresponding
   * to keys involved in this factor
   * @return vector of cameras
   */
  typename Base::Cameras cameras(const Values& values) const override {
    typename Base::Cameras cameras;
    for (size_t i = 0; i < this->keys_.size(); i++) {
      const Pose3& body_P_cam_i = body_P_sensors_[i];
      const Pose3 world_P_sensor_i = values.at<Pose3>(this->keys_[i])
          * body_P_cam_i;
      cameras.emplace_back(world_P_sensor_i, K_all_[i]);
    }
    return cameras;
  }

 private:

  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar & BOOST_SERIALIZATION_NVP(K_all_);
    ar & BOOST_SERIALIZATION_NVP(body_P_sensors_);
  }

};
// end of class declaration

/// traits
template<class CAMERA>
struct traits<SmartProjectionFactorP<CAMERA> > : public Testable<
    SmartProjectionFactorP<CAMERA> > {
};

}  // \ namespace gtsam
