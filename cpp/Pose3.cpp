/**
 * @file  Pose3.cpp
 * @brief 3D Pose
 */

#include "Pose3.h"

namespace gtsam {

/* ************************************************************************* */
Pose3 Pose3::exmap(const Vector& v) const { 
  return Pose3(R_.exmap(sub(v,0,3)), t_.exmap(sub(v,3,6)));
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
		Point3 sub = p - pose.translation();
		Point3 r = rotate(pose.rotation().inverse(), sub);
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
bool Pose3::equals(const Pose3& pose, double tol) const
{
  return R_.equals(pose.rotation(),tol) && t_.equals(pose.translation(),tol);
}

/* ************************************************************************* */
bool assert_equal(const Pose3& A, const Pose3& B, double tol)
{
  if(A.equals(B,tol)) return true;
  printf("not equal:\n");
  A.print("A");
  B.print("B");
  return false;
}

/* ************************************************************************* */
Pose3 Pose3::transformPose_to(const Pose3& transform)
{
		Rot3 cRv = rotation() * Rot3(transform.rotation().inverse());
		Point3 t = transform_to(transform, translation());
		
		return Pose3(cRv, t);
}

/* ************************************************************************* */
Pose3 composeTransform(const Pose3& current, const Pose3& target)
{
		// reverse operation
		Rot3 trans_rot = Rot3(target.rotation() * current.rotation().inverse()).inverse();

		// get sub
		Point3 sub = rotate(trans_rot, target.translation());
		
		// get final transform translation
		Point3 trans_pt = current.translation() - sub;
		
		return Pose3(trans_rot, trans_pt);
}

} // namespace gtsam
