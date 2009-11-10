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
VSLAMFactor::VSLAMFactor(const Vector& z, double sigma, int cn, int ln, const Cal3_S2 &K)
  : NonlinearFactor<VSLAMConfig>(z, sigma)
{
  cameraFrameNumber_ = cn;
  landmarkNumber_ = ln;
  char temp[100];
  temp[0] = 0;
  sprintf(temp, "x%d", cameraFrameNumber_);
  cameraFrameName_ = string(temp);
  temp[0] = 0;
  sprintf(temp, "l%d", landmarkNumber_);
  landmarkName_ = string(temp);
  K_ = K;
}

/* ************************************************************************* */
void VSLAMFactor::print(const std::string& s) const {
  printf("%s %s %s\n", s.c_str(), cameraFrameName_.c_str(), landmarkName_.c_str());
  gtsam::print(this->z_, s+".z");
}

/* ************************************************************************* */
bool VSLAMFactor::equals(const NonlinearFactor<VSLAMConfig>& f, double tol) const {
  const VSLAMFactor* p = dynamic_cast<const VSLAMFactor*>(&f);
  if (p == NULL) return false;
  if (cameraFrameNumber_ != p->cameraFrameNumber_ || landmarkNumber_ != p->landmarkNumber_) return false;
  if (!equal_with_abs_tol(this->z_,p->z_,tol)) return false;
  return true;
}

/* ************************************************************************* */
Vector VSLAMFactor::predict(const VSLAMConfig& c) const {
  Pose3 pose = c.cameraPose(cameraFrameNumber_);
  Point3 landmark = c.landmarkPoint(landmarkNumber_);
  return project(SimpleCamera(K_,pose), landmark).vector();
}

/* ************************************************************************* */
Vector VSLAMFactor::error_vector(const VSLAMConfig& c) const {
  // Right-hand-side b = (z - h(x))/sigma
  Vector h = predict(c);
  return (this->z_ - h);
}

/* ************************************************************************* */
LinearFactor::shared_ptr VSLAMFactor::linearize(const VSLAMConfig& c) const
{
  // get arguments from config
  Pose3 pose = c.cameraPose(cameraFrameNumber_);
  Point3 landmark = c.landmarkPoint(landmarkNumber_);

  SimpleCamera camera(K_,pose);

  // Jacobians
  Matrix Dh1 = Dproject_pose(camera, landmark);
  Matrix Dh2 = Dproject_point(camera, landmark);

  // Right-hand-side b = (z - h(x))
  Vector h = project(camera, landmark).vector();
  Vector b = this->z_ - h;

  // Make new linearfactor, divide by sigma
  LinearFactor::shared_ptr
    p(new LinearFactor(cameraFrameName_, Dh1, landmarkName_, Dh2, b, this->sigma_));
  return p;
}

/* ************************************************************************* */
} // namespace gtsam
