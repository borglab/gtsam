/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file CageFactor.h
 * @author Krunal Chande
 * @date November 10, 2014
 */

#pragma once

#include <boost/lexical_cast.hpp>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

/**
 * Factor to constrain position based on size of the accessible area
 */

class CageFactor: public NoiseModelFactor1<Pose3> {
private:
  Pose3 pose_;
  double cageBoundary_;
  typedef CageFactor This;
  typedef NoiseModelFactor1<Pose3> Base;

public:
  CageFactor() {} /* Default Constructor*/
  CageFactor(Key poseKey, const Pose3& pose, double cageBoundary, const SharedNoiseModel& model):
    Base(model, poseKey), pose_(pose), cageBoundary_(cageBoundary){}
  virtual ~CageFactor(){}

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** h(x) - z */
  Vector evaluateError(const Pose3& pose, boost::optional<Matrix&> H = boost::none) const {

        double distance = pose.translation().dist(Point3(0,0,0));
        if(distance > cageBoundary_){
          double distance = pose.range(Point3(0,0,0), H);
          return (gtsam::Vector(1) << distance - cageBoundary_);
        } else {
          if(H) *H = gtsam::zeros(1, Pose3::Dim());
          return (gtsam::Vector(1) << 0.0);
        }
//    Point3 p2;
//    double x = pose.x(), y = pose.y(), z = pose.z();
//    if (x < 0) x = -x;
//    if (y < 0) y = -y;
//    if (z < 0) z = -z;
//    double errorX = 100/(x-cageBoundary_), errorY = 100/(y-cageBoundary_), errorZ = 100/(z-cageBoundary_);
//    if (H) *H = pose.translation().distance(p2, H);
//    return (Vector(3)<<errorX, errorY, errorZ);
  }

  /** equals specialized to this factor */
  virtual bool equals(const NonlinearFactor& expected, double tol=1e-9) const {
    const This *e = dynamic_cast<const This*> (&expected);
    return e != NULL
        && Base::equals(*e, tol)
    ;
  }

  /** print contents */
  void print(const std::string& s="", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
    std::cout << s << "Cage Factor, Cage Boundary = " << cageBoundary_ << " Pose: " << pose_ << std::endl;
    Base::print("", keyFormatter);
  }

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & boost::serialization::make_nvp("NoiseModelFactor1",
        boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(cageBoundary_);
    ar & BOOST_SERIALIZATION_NVP(pose_);
  }

}; // end CageFactor
} // end namespace


