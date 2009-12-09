/**
 * @file    VSLAMFactor.cpp
 * @brief   A non-linear factor specialized to the Visual SLAM problem
 * @author  Alireza Fathi
 */

#include "VSLAMConfig.h"
#include "VSLAMFactor.h"
#include "Pose3.h"
#include "SimpleCamera.h"

using namespace std;
namespace gtsam{

/* ************************************************************************* */
VSLAMFactor::VSLAMFactor() {
	/// Arbitrary values
	cameraFrameNumber_ = 111;
	landmarkNumber_    = 222;
	cameraFrameName_ = symbol('x',cameraFrameNumber_);
	landmarkName_    = symbol('l',landmarkNumber_);
  keys_.push_back(cameraFrameName_);
  keys_.push_back(landmarkName_);
	K_ = shared_ptrK(new Cal3_S2(444,555,666,777,888));
}
/* ************************************************************************* */
VSLAMFactor::VSLAMFactor(const Point2& z, double sigma, int cn, int ln, const shared_ptrK &K)
  : NonlinearFactor<VSLAMConfig>(z.vector(), sigma)
{
  cameraFrameNumber_ = cn;
  landmarkNumber_    = ln;
  cameraFrameName_ = symbol('x',cameraFrameNumber_);
  landmarkName_    = symbol('l',landmarkNumber_);
  keys_.push_back(cameraFrameName_);
  keys_.push_back(landmarkName_);
  K_ = K;
}

/* ************************************************************************* */
void VSLAMFactor::print(const std::string& s) const {
  printf("%s %s %s\n", s.c_str(), cameraFrameName_.c_str(), landmarkName_.c_str());
  gtsam::print(this->z_, s+".z");
}

/* ************************************************************************* */
bool VSLAMFactor::equals(const VSLAMFactor& p, double tol) const {
  if (&p == NULL) return false;
  if (cameraFrameNumber_ != p.cameraFrameNumber_ || landmarkNumber_ != p.landmarkNumber_) return false;
  if (!equal_with_abs_tol(this->z_,p.z_,tol)) return false;
  return true;
}

/* ************************************************************************* */
Vector VSLAMFactor::predict(const VSLAMConfig& c) const {
  Pose3 pose = c.cameraPose(cameraFrameNumber_);
  Point3 landmark = c.landmarkPoint(landmarkNumber_);
  return project(SimpleCamera(*K_,pose), landmark).vector();
}

/* ************************************************************************* */
Vector VSLAMFactor::error_vector(const VSLAMConfig& c) const {
  // Right-hand-side b = (z - h(x))/sigma
  Vector h = predict(c);
  return (this->z_ - h);
}

/* ************************************************************************* */
GaussianFactor::shared_ptr VSLAMFactor::linearize(const VSLAMConfig& c) const
{
  // get arguments from config
  Pose3 pose = c.cameraPose(cameraFrameNumber_);
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
    p(new GaussianFactor(cameraFrameName_, Dh1, landmarkName_, Dh2, b, this->sigma_));
  return p;
}

/* ************************************************************************* */
} // namespace gtsam
