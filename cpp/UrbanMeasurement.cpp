/**
 * @file    UrbanMeasurement.cpp
 * @brief   A non-linear factor specialized to the Visual SLAM problem
 * @author  Frank Dellaert
 * @author  Viorela Ila
 */

#include "UrbanConfig.h"
#include "UrbanMeasurement.h"
#include "Pose3.h"
#include "SimpleCamera.h"

using namespace std;
namespace gtsam{
/** receives the 2D point in world coordinates and transforms it to Pose coordinates */
Point3 transformPoint2_to(const Pose3& pose, const Point2& p);
Matrix DtransformPoint2_to1(const Pose3& pose, const Point2& p);
Matrix DtransformPoint2_to2(const Pose3& pose); // does not depend on p !
/* ************************************************************************* */
UrbanMeasurement::UrbanMeasurement() {
	/// Arbitrary values
	robotPoseNumber_ = 111;
	landmarkNumber_    = 222;
	robotPoseName_ = symbol('x',robotPoseNumber_);
	landmarkName_    = symbol('l',landmarkNumber_);
	keys_.push_back(robotPoseName_);
	keys_.push_back(landmarkName_);
}
/* ************************************************************************* */
UrbanMeasurement::UrbanMeasurement(const Point2& z, double sigma, int cn, int ln)
: NonlinearFactor<UrbanConfig>(z.vector(), sigma)
  {
	robotPoseNumber_ = cn;
	landmarkNumber_    = ln;
	robotPoseName_ = symbol('x',robotPoseNumber_);
	landmarkName_    = symbol('l',landmarkNumber_);
	keys_.push_back(robotPoseName_);
	keys_.push_back(landmarkName_);
  }

/* ************************************************************************* */
void UrbanMeasurement::print(const std::string& s) const {
  printf("%s %s %s\n", s.c_str(), robotPoseName_.c_str(), landmarkName_.c_str());
  gtsam::print(this->z_, s+".z");
}

/* ************************************************************************* */
bool UrbanMeasurement::equals(const UrbanMeasurement& p, double tol) const {
  if (&p == NULL) return false;
  if (robotPoseNumber_ != p.robotPoseNumber_ || landmarkNumber_ != p.landmarkNumber_) return false;
  if (!equal_with_abs_tol(this->z_,p.z_,tol)) return false;
  return true;
}


// the difference here that we have a 2d point in a 3D coordinate
Vector UrbanMeasurement::predict(const UrbanConfig& c) const {
  Pose3 pose = c.robotPose(robotPoseNumber_);
  Point2 landmark = c.landmarkPoint(landmarkNumber_);
  // TODO Implement predict function
  Vector v;
  return v;
}

/* ************************************************************************* */
Vector UrbanMeasurement::error_vector(const UrbanConfig& c) const {
  // Right-hand-side b = (z - h(x))/sigma
  Point2 h = predict(c);
  // TODO Return z-h(x)
  Vector v;
  return v;
}

/* ************************************************************************* */

} // namespace gtsam
