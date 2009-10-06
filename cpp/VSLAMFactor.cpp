/**
 * @file    VSLAMFactor.cpp
 * @brief   A non-linear factor specialized to the Visual SLAM problem
 * @author  Alireza Fathi
 */

#include "VSLAMFactor.h"
#include "SimpleCamera.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
VSLAMFactor::VSLAMFactor(const Vector& z, double sigma, int cn, int ln, const Cal3_S2 &K)
  : NonlinearFactor<FGConfig>(z, sigma)
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
  ::print(z_, s+".z");
}

/* ************************************************************************* */
Vector VSLAMFactor::error_vector(const FGConfig& c) const {
  Pose3  pose     = c[cameraFrameName_];
  Point3 landmark = c[landmarkName_];

  // Right-hand-side b = (z - h(x))/sigma
  Vector h = project(SimpleCamera(K_,pose), landmark).vector();

  return (z_ - h);
}

/* ************************************************************************* */
LinearFactor::shared_ptr VSLAMFactor::linearize(const FGConfig& c) const
{
  // get arguments from config
  Pose3  pose     = c[cameraFrameName_]; // cast from Vector to Pose3 !!!
  Point3 landmark = c[landmarkName_];    // cast from Vector to Point3 !!

  SimpleCamera camera(K_,pose);

  // Jacobians
  Matrix Dh1 = Dproject_pose(camera, landmark);
  Matrix Dh2 = Dproject_point(camera, landmark);

  // Right-hand-side b = (z - h(x))
  Vector h = project(camera, landmark).vector();
  Vector b = z_ - h;

  // Make new linearfactor, divide by sigma
  LinearFactor::shared_ptr
    p(new LinearFactor(cameraFrameName_, Dh1/sigma_, landmarkName_, Dh2/sigma_, b/sigma_));
  return p;
}

/* ************************************************************************* */
bool VSLAMFactor::equals(const NonlinearFactor<FGConfig>& f, double tol) const {
  const VSLAMFactor* p = dynamic_cast<const VSLAMFactor*>(&f);
  if (p == NULL) goto fail;
  if (cameraFrameNumber_ != p->cameraFrameNumber_ || landmarkNumber_ != p->landmarkNumber_) goto fail;
  if (!equal_with_abs_tol(z_,p->z_,tol)) goto fail;
  return true;

 fail:
  print("actual");
  p->print("expected");
  return false;
}

/* ************************************************************************* */
string VSLAMFactor::dump() const
{
  int i = getCameraFrameNumber();
  int j = getLandmarkNumber();
  Vector z = measurement();
  char buffer[200];
  buffer[0] = 0;
  sprintf(buffer, "1 %d %d %f %d", i, j , sigma(), (int)z.size());
  for(size_t i = 0; i < z.size(); i++)
    sprintf(buffer, "%s %f", buffer, z(i));
  sprintf(buffer, "%s %s", buffer, K_.dump().c_str());

  return string(buffer);
	string temp;
	return temp;
}

