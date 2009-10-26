/**
 * @file  Pose3.cpp
 * @brief 3D Pose
 */

#include "Pose3.h"

using namespace std;

namespace gtsam {

/* ************************************************************************* */
void Pose3::print(const string& s) const {
	R_.print(s + ".R");
	t_.print(s + ".t");
}

/* ************************************************************************* */
bool Pose3::equals(const Pose3& pose, double tol) const
{
  return R_.equals(pose.R_,tol) && t_.equals(pose.t_,tol);
}

/* ************************************************************************* */
// Agrawal06iros, formula (6), seems to suggest this could be wrong:
Pose3 Pose3::exmap(const Vector& v) const { 
  return Pose3(R_.exmap(sub(v,0,3)), t_.exmap(sub(v,3,6)));
}

/* ************************************************************************* */
Vector Pose3::vector() const {
	Vector r = R_.vector(), t = t_.vector();
	return concatVectors(2, &r, &t);
}

/* ************************************************************************* */
Matrix Pose3::matrix() const {
	const double row4[] = { 0, 0, 0, 1 };
	Matrix A34 = Matrix_(3, 4, vector()), A14 = Matrix_(1, 4, row4);
	return stack(2, &A34, &A14);
}

/* ************************************************************************* */
Point3 transform_from(const Pose3& pose, const Point3& p) {
  return pose.R_ * p + pose.t_;
}
/* ************************************************************************* */
/** 3by6                                                                     */
/* ************************************************************************* */
Matrix Dtransform_from1(const Pose3& pose, const Point3& p) {
  Matrix DR = Drotate1(pose.rotation(), p);
  Matrix Dt = Dadd1(pose.translation(), p);
  return collect(2,&DR,&Dt);
}
/* ************************************************************************* */
/** 3by3                                                                     */
/* ************************************************************************* */
Matrix Dtransform_from2(const Pose3& pose) {
  return Drotate2(pose.rotation());
}

/* ************************************************************************* */
Point3 transform_to(const Pose3& pose, const Point3& p) {
		Point3 sub = p - pose.t_;
		Point3 r = unrotate(pose.R_, sub);
		return r; 
}
/* ************************************************************************* */
/** 3by6                                                                     */
/* ************************************************************************* */
Matrix Dtransform_to1(const Pose3& pose, const Point3& p) {
  Point3 q = p - pose.translation();
  Matrix D_r_R =   Dunrotate1(pose.rotation(),q);
  Matrix D_r_t = - Dunrotate2(pose.rotation()); // negative because of sub

  Matrix D_r_pose = collect(2,&D_r_R,&D_r_t);
  return D_r_pose;
}
/* ************************************************************************* */
/** 3by3                                                                     */
/* ************************************************************************* */
Matrix Dtransform_to2(const Pose3& pose) {
  return Dunrotate2(pose.rotation());
}

/* ************************************************************************* */
/** direct measurement of the deviation of a pose from the origin
 * used as soft prior
 */
/* ************************************************************************* */
Vector hPose (const Vector& x) {
  Pose3 pose(x);                            // transform from vector to Pose3
  Vector w = RQ(pose.rotation().matrix()); // get angle differences
  Vector d = pose.translation().vector();  // get translation differences
  return concatVectors(2,&w,&d);
}

/* ************************************************************************* */
/** derivative of direct measurement
 * 6*6, entry i,j is how measurement error will change 
 */
/* ************************************************************************* */
Matrix DhPose(const Vector& x) {
  Matrix H = eye(6,6);
  return H;
}

/* ************************************************************************* */
Pose3 Pose3::inverse() const
{
  Rot3 Rt = R_.inverse();
  return Pose3(Rt,-(Rt*t_));
}

/* ************************************************************************* */
Pose3 Pose3::transformPose_to(const Pose3& pose) const
{
		Rot3 cRv = R_ * Rot3(pose.R_.inverse());
		Point3 t = transform_to(pose, t_);
		
		return Pose3(cRv, t);
}
/* ************************************************************************* */

} // namespace gtsam
