/**
 * @file PoseRTV.cpp
 * @author Alex Cunningham
 */

#include <gtsam_unstable/dynamics/PoseRTV.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/base/Vector.h>

#include <cassert>

namespace gtsam {

using namespace std;

static const Vector kGravity = Vector::Unit(3,2)*9.81;

/* ************************************************************************* */
double bound(double a, double min, double max) {
  if (a < min) return min;
  else if (a > max) return max;
  else return a;
}

/* ************************************************************************* */
PoseRTV::PoseRTV(double roll, double pitch, double yaw, double x, double y,
    double z, double vx, double vy, double vz) :
    Base(Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z)),
        Velocity3(vx, vy, vz)) {
}

/* ************************************************************************* */
PoseRTV::PoseRTV(const Vector& rtv) :
    Base(Pose3(Rot3::RzRyRx(rtv.head(3)), Point3(rtv.segment(3, 3))),
        Velocity3(rtv.tail(3))) {
}

/* ************************************************************************* */
Vector PoseRTV::vector() const {
  Vector rtv(9);
  rtv.head(3) = rotation().xyz();
  rtv.segment(3,3) = translation();
  rtv.tail(3) = velocity();
  return rtv;
}

/* ************************************************************************* */
bool PoseRTV::equals(const PoseRTV& other, double tol) const {
  return pose().equals(other.pose(), tol)
      && equal_with_abs_tol(velocity(), other.velocity(), tol);
}

/* ************************************************************************* */
void PoseRTV::print(const string& s) const {
  cout << s << ":" << endl;
  gtsam::print((Vector)R().xyz(), "  R:rpy");
  cout << "  T" << t().transpose() << endl;
  gtsam::print((Vector)velocity(), "  V");
}

/* ************************************************************************* */
PoseRTV PoseRTV::planarDynamics(double vel_rate, double heading_rate,
    double max_accel, double dt) const {

  // split out initial state
  const Rot3& r1 = R();
  const Velocity3& v1 = v();

  // Update vehicle heading
  Rot3 r2 = r1.retract((Vector(3) << 0.0, 0.0, heading_rate * dt).finished());
  const double yaw2 = r2.ypr()(0);

  // Update vehicle position
  const double mag_v1 = v1.norm();

  // FIXME: this doesn't account for direction in velocity bounds
  double dv = bound(vel_rate - mag_v1, - (max_accel * dt), max_accel * dt);
  double mag_v2 = mag_v1 + dv;
  Velocity3 v2 = mag_v2 * Velocity3(cos(yaw2), sin(yaw2), 0.0);

  Point3 t2 = translationIntegration(r2, v2, dt);

  return PoseRTV(r2, t2, v2);
}

/* ************************************************************************* */
PoseRTV PoseRTV::flyingDynamics(
    double pitch_rate, double heading_rate, double lift_control, double dt) const {
  // split out initial state
  const Rot3& r1 = R();
  const Velocity3& v1 = v();

  // Update vehicle heading (and normalise yaw)
  Vector rot_rates = (Vector(3) << 0.0, pitch_rate, heading_rate).finished();
  Rot3 r2 = r1.retract(rot_rates*dt);

  // Work out dynamics on platform
  const double thrust = 50.0;
  const double lift   = 50.0;
  const double drag   = 0.1;
  double yaw2 = r2.yaw();
  double pitch2 = r2.pitch();
  double forward_accel = -thrust * sin(pitch2); // r2, pitch (in global frame?) controls forward force
  double loss_lift = lift*std::abs(sin(pitch2));
  Rot3 yaw_correction_bn = Rot3::Yaw(yaw2);
  Point3 forward(forward_accel, 0.0, 0.0);
  Vector Acc_n =
      yaw_correction_bn.rotate(forward)              // applies locally forward force in the global frame
      - drag * (Vector(3) << v1.x(), v1.y(), 0.0).finished()  // drag term dependent on v1
      + Vector::Unit(3,2)*(loss_lift - lift_control);                // falling due to lift lost from pitch

  // Update Vehicle Position and Velocity
  Velocity3 v2 = v1 + Velocity3(Acc_n * dt);
  Point3 t2 = translationIntegration(r2, v2, dt);

  return PoseRTV(r2, t2, v2);
}

/* ************************************************************************* */
PoseRTV PoseRTV::generalDynamics(
    const Vector& accel, const Vector& gyro, double dt) const {
  //  Integrate Attitude Equations
  Rot3 r2 = rotation().retract(gyro * dt);

  //  Integrate Velocity Equations
  Velocity3 v2 = velocity() + Velocity3(dt * (r2.matrix() * accel + kGravity));

  //  Integrate Position Equations
  Point3 t2 = translationIntegration(r2, v2, dt);

  return PoseRTV(t2, r2, v2);
}

/* ************************************************************************* */
Vector6 PoseRTV::imuPrediction(const PoseRTV& x2, double dt) const {
  // split out states
  const Rot3      &r1 = R(), &r2 = x2.R();
  const Velocity3 &v1 = v(), &v2 = x2.v();

  Vector6 imu;

  // acceleration
  Vector3 accel = (v2-v1) / dt;
  imu.head<3>() = r2.transpose() * (accel - kGravity);

  // rotation rates
  // just using euler angles based on matlab code
  // FIXME: this is silly - we shouldn't use differences in Euler angles
  Matrix Enb = RRTMnb(r1);
  Vector3 euler1 = r1.xyz(), euler2 = r2.xyz();
  Vector3 dR = euler2 - euler1;

  // normalize yaw in difference (as per Mitch's code)
  dR(2) = Rot2::fromAngle(dR(2)).theta();
  dR /= dt;
  imu.tail<3>() = Enb * dR;
//  imu.tail(3) = r1.transpose() * dR;

  return imu;
}

/* ************************************************************************* */
Point3 PoseRTV::translationIntegration(const Rot3& r2, const Velocity3& v2, double dt) const {
  // predict point for constraint
  // NOTE: uses simple Euler approach for prediction
  Point3 pred_t2 = t() + Point3(v2 * dt);
  return pred_t2;
}

/* ************************************************************************* */
double PoseRTV::range(const PoseRTV& other,
    OptionalJacobian<1,9> H1, OptionalJacobian<1,9> H2) const {
  Matrix36 D_t1_pose, D_t2_other;
  const Point3 t1 = pose().translation(H1 ? &D_t1_pose : 0);
  const Point3 t2 = other.pose().translation(H2 ? &D_t2_other : 0);
  Matrix13 D_d_t1, D_d_t2;
  double d = distance3(t1, t2, H1 ? &D_d_t1 : 0, H2 ? &D_d_t2 : 0);
  if (H1) *H1 << D_d_t1 * D_t1_pose, 0,0,0;
  if (H2) *H2 << D_d_t2 * D_t2_other, 0,0,0;
  return d;
}

/* ************************************************************************* */
PoseRTV PoseRTV::transformed_from(const Pose3& trans, ChartJacobian Dglobal,
    OptionalJacobian<9, 6> Dtrans) const {

  // Pose3 transform is just compose
  Matrix6 D_newpose_trans, D_newpose_pose;
  Pose3 newpose = trans.compose(pose(), D_newpose_trans, D_newpose_pose);

  // Note that we rotate the velocity
  Matrix3 D_newvel_R, D_newvel_v;
  Velocity3 newvel = trans.rotation().rotate(Point3(velocity()), D_newvel_R, D_newvel_v);

  if (Dglobal) {
    Dglobal->setZero();
    Dglobal->topLeftCorner<6,6>() = D_newpose_pose;
    Dglobal->bottomRightCorner<3,3>() = D_newvel_v;
  }

  if (Dtrans) {
    Dtrans->setZero();
    Dtrans->topLeftCorner<6,6>() = D_newpose_trans;
    Dtrans->bottomLeftCorner<3,3>() = D_newvel_R;
  }
  return PoseRTV(newpose, newvel);
}

/* ************************************************************************* */
Matrix PoseRTV::RRTMbn(const Vector3& euler) {
  assert(euler.size() == 3);
  const double s1 = sin(euler.x()), c1 = cos(euler.x());
  const double t2 = tan(euler.y()), c2 = cos(euler.y());
  Matrix Ebn(3,3);
  Ebn << 1.0, s1 * t2, c1 * t2,
         0.0,      c1,     -s1,
         0.0, s1 / c2, c1 / c2;
  return Ebn;
}

/* ************************************************************************* */
Matrix PoseRTV::RRTMbn(const Rot3& att) {
  return PoseRTV::RRTMbn(att.rpy());
}

/* ************************************************************************* */
Matrix PoseRTV::RRTMnb(const Vector3& euler) {
  Matrix Enb(3,3);
  const double s1 = sin(euler.x()), c1 = cos(euler.x());
  const double s2 = sin(euler.y()), c2 = cos(euler.y());
  Enb << 1.0, 0.0,   -s2,
         0.0,  c1, s1*c2,
         0.0, -s1, c1*c2;
  return Enb;
}

/* ************************************************************************* */
Matrix PoseRTV::RRTMnb(const Rot3& att) {
  return PoseRTV::RRTMnb(att.rpy());
}

/* ************************************************************************* */
} // \namespace gtsam
