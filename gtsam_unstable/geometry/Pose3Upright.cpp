/**
 * @file Pose3Upright.cpp
 *
 * @date Jan 24, 2012
 * @author Alex Cunningham
 */

#include <iostream>
#include "gtsam/base/OptionalJacobian.h"
#include "gtsam/base/Vector.h"

#include <gtsam_unstable/geometry/Pose3Upright.h>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
Pose3Upright::Pose3Upright(const Rot2& bearing, const Point3& t)
: T_(bearing, Point2(t.x(), t.y())), z_(t.z())
{
}

/* ************************************************************************* */
Pose3Upright::Pose3Upright(double x, double y, double z, double theta)
: T_(x, y, theta), z_(z)
{
}

/* ************************************************************************* */
Pose3Upright::Pose3Upright(const Pose2& pose, double z)
: T_(pose), z_(z)
{
}

/* ************************************************************************* */
Pose3Upright::Pose3Upright(const Pose3& x)
: T_(x.x(), x.y(), x.rotation().yaw()), z_(x.z())
{
}

/* ************************************************************************* */
void Pose3Upright::print(const std::string& s) const {
  cout << s << "(" << T_.x() << ", " << T_.y() << ", " << z_ << ", " << T_.theta() << ")" << endl;
}

/* ************************************************************************* */
bool Pose3Upright::equals(const Pose3Upright& x, double tol) const {
  return T_.equals(x.T_, tol) && std::abs(z_ - x.z_) < tol;
}

/* ************************************************************************* */
Point3 Pose3Upright::translation() const {
  return Point3(x(), y(), z());
}

/* ************************************************************************* */
Point2 Pose3Upright::translation2() const {
  return T_.t();
}

/* ************************************************************************* */
Rot2 Pose3Upright::rotation2() const {
  return T_.r();
}

/* ************************************************************************* */
Rot3 Pose3Upright::rotation() const {
  return Rot3::Yaw(theta());
}

/* ************************************************************************* */
Pose2 Pose3Upright::pose2() const {
  return T_;
}

/* ************************************************************************* */
Pose3 Pose3Upright::pose() const {
  return Pose3(rotation(), translation());
}

/* ************************************************************************* */
Pose3Upright Pose3Upright::inverse(OptionalJacobian<4, 4> H1) const {
  if (!H1) {
    return Pose3Upright(T_.inverse(), -z_);
  }
  OptionalJacobian<3, 3>::Jacobian H3x3;
  // TODO(kartikarcot): Could not use reference to a view into H1 and reuse memory
  // Eigen::Ref<Eigen::Matrix<double, 3, 3>> H3x3 = H1->topLeftCorner(3,3);
  Pose3Upright result(T_.inverse(H3x3), -z_);
  Matrix H1_ = -I_4x4;
  H1_.topLeftCorner(2, 2) = H3x3.topLeftCorner(2, 2);
  H1_.topRightCorner(2, 1) = H3x3.topRightCorner(2, 1);
  *H1 = H1_;
  return result;
}

/* ************************************************************************* */
Pose3Upright Pose3Upright::compose(const Pose3Upright& p2,
    OptionalJacobian<4,4> H1, OptionalJacobian<4,4> H2) const {
  if (!H1 && !H2)
    return Pose3Upright(T_.compose(p2.T_), z_ + p2.z_);

  // TODO(kartikarcot): Could not use reference to a view into H1 and reuse memory
  OptionalJacobian<3, 3>::Jacobian H3x3;
  Pose3Upright result(T_.compose(p2.T_, H3x3), z_ + p2.z_);
  if (H1) {
    Matrix H1_ = I_4x4;
    H1_.topLeftCorner(2,2) = H3x3.topLeftCorner(2,2);
    H1_.topRightCorner(2, 1) = H3x3.topRightCorner(2, 1);
    *H1 = H1_;
  }
  if (H2) *H2 = I_4x4;
  return result;
}

/* ************************************************************************* */
Pose3Upright Pose3Upright::between(const Pose3Upright& p2,
    OptionalJacobian<4,4> H1, OptionalJacobian<4,4> H2) const {
  if (!H1 && !H2)
    return Pose3Upright(T_.between(p2.T_), p2.z_ - z_);

  // TODO(kartikarcot): Could not use reference to a view into H1 and H2 to reuse memory
  OptionalJacobian<3, 3>::Jacobian H3x3_1, H3x3_2;
  Pose3Upright result(T_.between(p2.T_, H3x3_1, H3x3_2), p2.z_ - z_);
  if (H1) {
    Matrix H1_ = -I_4x4;
    H1_.topLeftCorner(2,2) = H3x3_1.topLeftCorner(2,2);
    H1_.topRightCorner(2, 1) = H3x3_1.topRightCorner(2, 1);
    *H1 = H1_;
  }
  if (H2) *H2 = I_4x4;
  return result;
}

/* ************************************************************************* */
Pose3Upright Pose3Upright::retract(const Vector& v) const {
  assert(v.size() == 4);
  Vector v1(3); v1 << v(0), v(1), v(3);
  return Pose3Upright(T_.retract(v1), z_ + v(2));
}

/* ************************************************************************* */
Vector Pose3Upright::localCoordinates(const Pose3Upright& p2) const {
  Vector pose2 = T_.localCoordinates(p2.pose2());
  Vector result(4);
  result << pose2(0), pose2(1), p2.z() - z_, pose2(2);
  return result;
}

/* ************************************************************************* */
Pose3Upright Pose3Upright::Expmap(const Vector& xi) {
  assert(xi.size() == 4);
  Vector v1(3); v1 << xi(0), xi(1), xi(3);
  return Pose3Upright(Pose2::Expmap(v1), xi(2));
}

/* ************************************************************************* */
Vector Pose3Upright::Logmap(const Pose3Upright& p) {
  Vector pose2 = Pose2::Logmap(p.pose2());
  Vector result(4);
  result << pose2(0), pose2(1), p.z(), pose2(2);
  return result;
}
/* ************************************************************************* */

} // \namespace gtsam



