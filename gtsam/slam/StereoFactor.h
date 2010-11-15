/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    StereoFactor.h
 * @brief   A non-linear factor for stereo measurements
 * @author  Alireza Fathi
 * @author  Chris Beall
 */

#pragma once

#include <gtsam/slam/visualSLAM.h>
#include <gtsam/geometry/Cal3_S2.h>

namespace gtsam {

using namespace gtsam;
using namespace gtsam::visualSLAM;

class StereoFactor: public NonlinearFactor2<Values, PoseKey, PointKey>, Testable<StereoFactor> {
private:

  // Keep a copy of measurement and calibration for I/O
  StereoPoint2 z_;
  boost::shared_ptr<Cal3_S2> K_;
  double baseline_;

public:

  // shorthand for base class type
  typedef NonlinearFactor2<Values, PoseKey, PointKey> Base;

  // shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<StereoFactor> shared_ptr;

  /**
   * Default constructor
   */
  StereoFactor() : K_(new Cal3_S2(444, 555, 666, 777, 888)) {}

  /**
   * Constructor
   * @param z is the 2 dimensional location of point in image (the measurement)
   * @param sigma is the standard deviation
   * @param cameraFrameNumber is basically the frame number
   * @param landmarkNumber is the index of the landmark
   * @param K the constant calibration
   */
  StereoFactor(const StereoPoint2& z, const SharedGaussian& model, PoseKey j_pose, PointKey j_landmark, const shared_ptrK& K, double baseline) :
  	Base(model, j_pose, j_landmark), z_(z), K_(K), baseline_(baseline) {}

  /**
   * print
   * @param s optional string naming the factor
   */
  void print(const std::string& s) const {
    printf("%s %s %s\n", s.c_str(), ((string) key1_).c_str(),
    ((string) key2_).c_str());
    z_.print(s+".z");
  }

  /**
   * equals
   */
  bool equals(const StereoFactor& f, double tol) const {
     const StereoFactor* p = dynamic_cast<const StereoFactor*>(&f);
     if (p == NULL) goto fail;
     //if (cameraFrameNumber_ != p->cameraFrameNumber_ || landmarkNumber_ != p->landmarkNumber_) goto fail;
     if (!z_.equals(p->z_,tol)) goto fail;
     return true;

    fail:
     print("actual");
     p->print("expected");
     return false;
   }

  /** h(x)-z */
  Vector evaluateError(const Pose3& pose, const Point3& point,
      boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {

  	const Cal3_S2& K = *K_;
    StereoCamera stereoCam(pose, K, baseline_);

    return (stereoCam.project(point,H1,H2) - z_).vector();
  }

  StereoPoint2 z(){return z_;}

private:
  /** Serialization function */
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version) {
    //ar & BOOST_SERIALIZATION_NVP(key1_);
    //ar & BOOST_SERIALIZATION_NVP(key2_);
    ar & BOOST_SERIALIZATION_NVP(z_);
    ar & BOOST_SERIALIZATION_NVP(K_);
  }
};

}
