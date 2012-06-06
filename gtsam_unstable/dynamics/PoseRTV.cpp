/**
 * @file PoseRTV.cpp
 * @author Alex Cunningham
 */

#include <gtsam/3rdparty/Eigen/Eigen/LU>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Lie-inl.h>
#include <gtsam/geometry/Pose2.h>

#include <gtsam_unstable/dynamics/PoseRTV.h>

namespace gtsam {

using namespace std;

static const Vector g = delta(3, 2, 9.81);
const double pi = M_PI;

/* ************************************************************************* */
double bound(double a, double min, double max) {
	if (a < min) return min;
	else if (a > max) return max;
	else return a;
}

/* ************************************************************************* */
PoseRTV::PoseRTV(double roll, double pitch, double yaw, double x, double y, double z,
		double vx, double vy, double vz)
: Rt_(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z)), v_(vx, vy, vz) {}

/* ************************************************************************* */
PoseRTV::PoseRTV(const Vector& rtv)
: Rt_(Rot3::RzRyRx(rtv.head(3)), Point3(rtv.segment(3, 3))), v_(rtv.tail(3))
{
}

/* ************************************************************************* */
Vector PoseRTV::vector() const {
	Vector rtv(9);
	rtv.head(3) = Rt_.rotation().xyz();
	rtv.segment(3,3) = Rt_.translation().vector();
	rtv.tail(3) = v_.vector();
	return rtv;
}

/* ************************************************************************* */
bool PoseRTV::equals(const PoseRTV& other, double tol) const {
	return Rt_.equals(other.Rt_, tol) && v_.equals(other.v_, tol);
}

/* ************************************************************************* */
void PoseRTV::print(const string& s) const {
	cout << s << ":" << endl;
	gtsam::print((Vector)R().xyz(), "  R:rpy");
	t().print("  T");
	v_.print("  V");
}

/* ************************************************************************* */
PoseRTV PoseRTV::Expmap(const Vector& v) {
	assert(v.size() == 9);
	Pose3 newPose = Pose3::Expmap(sub(v, 0, 6));
	Velocity3 newVel = Velocity3::Expmap(sub(v, 6, 9));
	return PoseRTV(newPose, newVel);
}

/* ************************************************************************* */
Vector PoseRTV::Logmap(const PoseRTV& p) {
	Vector Lx = Pose3::Logmap(p.Rt_);
	Vector Lv = Velocity3::Logmap(p.v_);
	return concatVectors(2, &Lx, &Lv);
}

/* ************************************************************************* */
PoseRTV PoseRTV::retract(const Vector& v) const {
	assert(v.size() == 9);
	// First order approximation
	Pose3 newPose = Rt_.retract(sub(v, 0, 6));
	Velocity3 newVel = v_ + Rt_.rotation() * Point3(sub(v, 6, 9));
	return PoseRTV(newPose, newVel);
}

/* ************************************************************************* */
Vector PoseRTV::localCoordinates(const PoseRTV& p1) const {
	const Pose3& x0 = pose(), &x1 = p1.pose();
	// First order approximation
	Vector poseLogmap = x0.localCoordinates(x1);
	Vector lv = rotation().unrotate(p1.velocity() - v_).vector();
	return concatVectors(2, &poseLogmap, &lv);
}

/* ************************************************************************* */
PoseRTV PoseRTV::inverse() const {
	return PoseRTV(Rt_.inverse(), v_.inverse());
}

/* ************************************************************************* */
PoseRTV PoseRTV::compose(const PoseRTV& p) const {
	return PoseRTV(Rt_.compose(p.Rt_), v_.compose(p.v_));
}

/* ************************************************************************* */
PoseRTV between_(const PoseRTV& p1, const PoseRTV& p2) { return p1.between(p2); }
PoseRTV PoseRTV::between(const PoseRTV& p,
		boost::optional<Matrix&> H1,
		boost::optional<Matrix&> H2) const {
	if (H1) *H1 = numericalDerivative21(between_, *this, p, 1e-5);
	if (H2) *H2 = numericalDerivative22(between_, *this, p, 1e-5);
	return inverse().compose(p);
}

/* ************************************************************************* */
PoseRTV PoseRTV::planarDynamics(double vel_rate, double heading_rate,
		double max_accel, double dt) const {

	// split out initial state
	const Rot3& r1 = R();
	const Velocity3& v1 = v();

	// Update vehicle heading
	Rot3 r2 = r1.retract(Vector_(3, 0.0, 0.0, heading_rate * dt));
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
	Vector rot_rates = Vector_(3, 0.0, pitch_rate, heading_rate);
	Rot3 r2 = r1.retract(rot_rates*dt);

	// Work out dynamics on platform
	const double thrust = 50.0;
	const double lift   = 50.0;
	const double drag   = 0.1;
	double yaw2 = r2.yaw();
	double pitch2 = r2.pitch();
	double forward_accel = -thrust * sin(pitch2); // r2, pitch (in global frame?) controls forward force
	double loss_lift = lift*fabs(sin(pitch2));
	Rot3 yaw_correction_bn = Rot3::yaw(yaw2);
	Point3 forward(forward_accel, 0.0, 0.0);
	Vector Acc_n =
			yaw_correction_bn.rotate(forward).vector()   // applies locally forward force in the global frame
			- drag * Vector_(3, v1.x(), v1.y(), 0.0)     // drag term dependent on v1
			+ delta(3, 2, loss_lift - lift_control);     // falling due to lift lost from pitch

	// Update Vehicle Position and Velocity
	Velocity3 v2 = v1 + Velocity3(Acc_n * dt);
	Point3 t2 = translationIntegration(r2, v2, dt);

	return PoseRTV(r2, t2, v2);
}

/* ************************************************************************* */
PoseRTV PoseRTV::generalDynamics(
		const Vector& accel, const Vector& gyro, double dt) const {
	//	Integrate Attitude Equations
	Rot3 r2 = rotation().retract(gyro * dt);

	//	Integrate Velocity Equations
	Velocity3 v2 = v_.compose(Velocity3(dt * (r2.matrix() * accel + g)));

	//	Integrate Position Equations
	Point3 t2 = translationIntegration(r2, v2, dt);

	return PoseRTV(t2, r2, v2);
}

/* ************************************************************************* */
Vector PoseRTV::imuPrediction(const PoseRTV& x2, double dt) const {
	// split out states
	const Rot3      &r1 = R(), &r2 = x2.R();
	const Velocity3 &v1 = v(), &v2 = x2.v();

	Vector imu(6);

	// acceleration
	Vector accel = v1.localCoordinates(v2) / dt;
	imu.head(3) = r2.transpose() * (accel - g);

	// rotation rates
	// just using euler angles based on matlab code
	// FIXME: this is silly - we shouldn't use differences in Euler angles
	Matrix Enb = RRTMnb(r1);
	Vector euler1 = r1.xyz(), euler2 = r2.xyz();
	Vector dR = euler2 - euler1;

	// normalize yaw in difference (as per Mitch's code)
	dR(2) = Rot2::fromAngle(dR(2)).theta();
	dR /= dt;
	imu.tail(3) = Enb * dR;
//	imu.tail(3) = r1.transpose() * dR;

	return imu;
}

/* ************************************************************************* */
Point3 PoseRTV::translationIntegration(const Rot3& r2, const Velocity3& v2, double dt) const {
	// predict point for constraint
	// NOTE: uses simple Euler approach for prediction
	Point3 pred_t2 = t() + v2 * dt;
	return pred_t2;
}

/* ************************************************************************* */
double range_(const PoseRTV& A, const PoseRTV& B) { return A.range(B); }
double PoseRTV::range(const PoseRTV& other,
		boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
	if (H1) *H1 = numericalDerivative21(range_, *this, other, 1e-5);
	if (H2) *H2 = numericalDerivative22(range_, *this, other, 1e-5);
	return t().dist(other.t());
}

/* ************************************************************************* */
PoseRTV transformed_from_(const PoseRTV& global, const Pose3& transform) {
	return global.transformed_from(transform);
}

PoseRTV PoseRTV::transformed_from(const Pose3& trans,
		boost::optional<Matrix&> Dglobal,
		boost::optional<Matrix&> Dtrans) const {
	// Note that we rotate the velocity
	Matrix DVr, DTt;
	Velocity3 newvel = trans.rotation().rotate(v_, DVr, DTt);
	if (!Dglobal && !Dtrans)
		return PoseRTV(trans.compose(pose()), newvel);

	// Pose3 transform is just compose
	Matrix DTc, DGc;
	Pose3 newpose = trans.compose(pose(), DTc, DGc);

	if (Dglobal) {
		*Dglobal = zeros(9,9);
		insertSub(*Dglobal, DGc, 0, 0);

		// Rotate velocity
		insertSub(*Dglobal, eye(3,3), 6, 6); // FIXME: should this actually be an identity matrix?
	}

	if (Dtrans) {
		*Dtrans = numericalDerivative22(transformed_from_, *this, trans, 1e-8);
		//
		//		*Dtrans = zeros(9,6);
		//		// directly affecting the pose
		//		insertSub(*Dtrans, DTc, 0, 0); // correct in tests
		//
		//		// rotating the velocity
		//		Matrix vRhat = skewSymmetric(-v_.x(), -v_.y(), -v_.z());
		//		trans.rotation().print("Transform rotation");
		//		gtsam::print(vRhat, "vRhat");
		//		gtsam::print(DVr, "DVr");
		//		// FIXME: find analytic derivative
		////		insertSub(*Dtrans, vRhat, 6, 0); // works if PoseRTV.rotation() = I
		////		insertSub(*Dtrans, trans.rotation().matrix() * vRhat, 6, 0); // FAIL: both tests fail
	}
	return PoseRTV(newpose, newvel);
}

/* ************************************************************************* */
Matrix PoseRTV::RRTMbn(const Vector& euler) {
	assert(euler.size() == 3);
	const double s1 = sin(euler(1-1)), c1 = cos(euler(1-1));
	const double t2 = tan(euler(2-1)), c2 = cos(euler(2-1));
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
Matrix PoseRTV::RRTMnb(const Vector& euler) {
	assert(euler.size() == 3);
	Matrix Enb(3,3);
	const double s1 = sin(euler(1-1)), c1 = cos(euler(1-1));
	const double s2 = sin(euler(2-1)), c2 = cos(euler(2-1));
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
