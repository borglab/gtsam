/**
 * @file    UrbanOdometryFactor.cpp
 * @brief   A non-linear factor specialized to the Visual SLAM problem
 * @author  Frank Dellaert
 * @author  Viorela Ila
 */

#include "Pose3Factor.h"

using namespace std;
namespace gtsam {

/* ************************************************************************* *
Pose3Factor::Pose3Factor() {
	/// Arbitrary values
	robotPoseNumber_ = 1;
	robotPoseName_ = symbol('x',robotPoseNumber_);
	keys_.push_back(robotPoseName_);
}
/* ************************************************************************* *
Pose3Factor::Pose3Factor(const Point2& z, double sigma, int cn, int ln)
: NonlinearFactor<Pose3Config>(z.vector(), sigma)
  {
	robotPoseNumber_ = cn;
	landmarkNumber_    = ln;
	robotPoseName_ = symbol('x',robotPoseNumber_);
	landmarkName_    = symbol('l',landmarkNumber_);
	keys_.push_back(robotPoseName_);
	keys_.push_back(landmarkName_);
  }

/* ************************************************************************* *
void Pose3Factor::print(const std::string& s) const {
  printf("%s %s %s\n", s.c_str(), robotPoseName_.c_str(), landmarkName_.c_str());
  gtsam::print(this->z_, s+".z");
}

/* ************************************************************************* *
bool Pose3Factor::equals(const Pose3Factor& p, double tol) const {
  if (&p == NULL) return false;
  if (robotPoseNumber_ != p.robotPoseNumber_ || landmarkNumber_ != p.landmarkNumber_) return false;
  if (!equal_with_abs_tol(this->z_,p.z_,tol)) return false;
  return true;
}

/* ************************************************************************* *
// TODO Implement transformPoint2_from
// the difference here that we have a 2d point in a 3D coordinate
Vector Pose3Factor::predict(const Pose3Config& c) const {
  Pose3 pose = c.cameraPose(robotPoseNumber_);
  Point2 landmark = c.landmarkPoint(landmarkNumber_);
  return transformPoint2_from(SimpleCamera(*K_,pose), landmark).vector();
}

/* ************************************************************************* *
Vector Pose3Factor::error_vector(const Pose3Config& c) const {
  // Right-hand-side b = (z - h(x))/sigma
  Point2 h = predict(c);
  return (this->z_ - h);
}

/* ************************************************************************* *
GaussianFactor::shared_ptr Pose3Factor::linearize(const Pose3Config& c) const
{
  // get arguments from config
  Pose3 pose = c.cameraPose(robotPoseNumber_);
  Point3 landmark = c.landmarkPoint(landmarkNumber_);

  SimpleCamera camera(*K_,pose);

  // Jacobians
  Matrix Dh1 = Dproject_pose(camera, landmark);
  Matrix Dh2 = Dproject_point(camera, landmark);

  // Right-hand-side b = (z - h(x))
  Vector h = project(camera, landmark).vector();
  Vector b = this->z_ - h;

  // Make new linearfactor, divide by sigma
  GaussianFactor::shared_ptr
    p(new GaussianFactor(robotPoseName_, Dh1, landmarkName_, Dh2, b, this->sigma_));
  return p;
}

/* ************************************************************************* */
} // namespace gtsam
