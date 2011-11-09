/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testRot3.cpp
 * @brief   Unit tests for Rot3Q class
 * @author  Alireza Fathi
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <boost/math/constants/constants.hpp>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/lieProxies.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3Q.h>

using namespace gtsam;

Rot3Q R = Rot3Q::rodriguez(0.1, 0.4, 0.2);
Point3 P(0.2, 0.7, -2.0);
double error = 1e-9, epsilon = 0.001;

/* ************************************************************************* */
TEST( Rot3Q, constructor)
{
	Rot3Q expected(eye(3, 3));
	Vector r1(3), r2(3), r3(3);
	r1(0) = 1;
	r1(1) = 0;
	r1(2) = 0;
	r2(0) = 0;
	r2(1) = 1;
	r2(2) = 0;
	r3(0) = 0;
	r3(1) = 0;
	r3(2) = 1;
	Rot3Q actual(r1, r2, r3);
	CHECK(assert_equal(actual,expected));
}

/* ************************************************************************* */
TEST( Rot3Q, constructor2)
{
	Matrix R = Matrix_(3, 3, 11., 12., 13., 21., 22., 23., 31., 32., 33.);
	Rot3Q actual(R);
	Rot3Q expected(11, 12, 13, 21, 22, 23, 31, 32, 33);
	CHECK(assert_equal(actual,expected));
}

/* ************************************************************************* */
TEST( Rot3Q, constructor3)
{
	Rot3Q expected(1, 2, 3, 4, 5, 6, 7, 8, 9);
	Point3 r1(1, 4, 7), r2(2, 5, 8), r3(3, 6, 9);
	CHECK(assert_equal(Rot3Q(r1,r2,r3),expected));
}

/* ************************************************************************* */
TEST( Rot3Q, equals)
{
	CHECK(R.equals(R));
	Rot3Q zero;
	CHECK(!R.equals(zero));
}

/* ************************************************************************* */
// Notice this uses J^2 whereas fast uses w*w', and has cos(t)*I + ....
Rot3Q slow_but_correct_rodriguez(const Vector& w) {
	double t = norm_2(w);
	Matrix J = skewSymmetric(w / t);
	if (t < 1e-5) return Rot3Q();
	Matrix R = eye(3) + sin(t) * J + (1.0 - cos(t)) * (J * J);
	return R;
}

/* ************************************************************************* */
TEST( Rot3Q, rodriguez)
{
	Rot3Q R1 = Rot3Q::rodriguez(epsilon, 0, 0);
	Vector w = Vector_(3, epsilon, 0., 0.);
	Rot3Q R2 = slow_but_correct_rodriguez(w);
  Rot3Q expected2(Rot3M::rodriguez(epsilon, 0, 0));
  CHECK(assert_equal(expected2,R1,1e-5));
	CHECK(assert_equal(R2,R1));
}

/* ************************************************************************* */
TEST( Rot3Q, rodriguez2)
{
	Vector axis = Vector_(3,0.,1.,0.); // rotation around Y
	double angle = 3.14 / 4.0;
	Rot3Q actual = Rot3Q::rodriguez(axis, angle);
	Rot3Q expected(0.707388, 0, 0.706825,
			                 0, 1,        0,
			         -0.706825, 0, 0.707388);
  Rot3Q expected2(Rot3M::rodriguez(axis, angle));
	CHECK(assert_equal(expected,actual,1e-5));
  CHECK(assert_equal(expected2,actual,1e-5));
}

/* ************************************************************************* */
TEST( Rot3Q, rodriguez3)
{
	Vector w = Vector_(3, 0.1, 0.2, 0.3);
	Rot3Q R1 = Rot3Q::rodriguez(w / norm_2(w), norm_2(w));
	Rot3Q R2 = slow_but_correct_rodriguez(w);
  Rot3Q expected2(Rot3M::rodriguez(w / norm_2(w), norm_2(w)));
  CHECK(assert_equal(expected2,R1));
	CHECK(assert_equal(R2,R1));
}

/* ************************************************************************* */
TEST( Rot3Q, rodriguez4)
{
	Vector axis = Vector_(3,0.,0.,1.); // rotation around Z
	double angle = M_PI_2;
	Rot3Q actual = Rot3Q::rodriguez(axis, angle);
	double c=cos(angle),s=sin(angle);
	Rot3Q expected1(c,-s, 0,
			          s, c, 0,
			          0, 0, 1);
	Rot3Q expected2(Rot3M::rodriguez(axis, angle));
	CHECK(assert_equal(expected1,actual,1e-5));
  CHECK(assert_equal(expected2,actual,1e-5));
	CHECK(assert_equal(slow_but_correct_rodriguez(axis*angle),actual,1e-5));
}

/* ************************************************************************* */
TEST( Rot3Q, expmap)
{
	Vector v = zero(3);
	CHECK(assert_equal(R.retract(v), R));
}

/* ************************************************************************* */
TEST(Rot3Q, log)
{
	Vector w1 = Vector_(3, 0.1, 0.0, 0.0);
	Rot3Q R1 = Rot3Q::rodriguez(w1);
	Rot3Q R1m = Rot3M::rodriguez(w1);
	CHECK(assert_equal(w1, Rot3Q::Logmap(R1)));
	CHECK(assert_equal(R1m, R1));

	Vector w2 = Vector_(3, 0.0, 0.1, 0.0);
	Rot3Q R2 = Rot3Q::rodriguez(w2);
  Rot3Q R2m = Rot3M::rodriguez(w2);
	CHECK(assert_equal(w2, Rot3Q::Logmap(R2)));
  CHECK(assert_equal(R2m, R2));

	Vector w3 = Vector_(3, 0.0, 0.0, 0.1);
	Rot3Q R3 = Rot3Q::rodriguez(w3);
  Rot3Q R3m = Rot3M::rodriguez(w3);
	CHECK(assert_equal(w3, Rot3Q::Logmap(R3)));
  CHECK(assert_equal(R3m, R3));

	Vector w = Vector_(3, 0.1, 0.4, 0.2);
	Rot3Q R = Rot3Q::rodriguez(w);
  Rot3Q Rm = Rot3M::rodriguez(w);
	CHECK(assert_equal(w, Rot3Q::Logmap(R)));
  CHECK(assert_equal(Rm, R));

	Vector w5 = Vector_(3, 0.0, 0.0, 0.0);
	Rot3Q R5 = Rot3Q::rodriguez(w5);
  Rot3Q R5m = Rot3M::rodriguez(w5);
	CHECK(assert_equal(w5, Rot3Q::Logmap(R5)));
  CHECK(assert_equal(R5m, R5));

	Vector w6 = Vector_(3, boost::math::constants::pi<double>(), 0.0, 0.0);
	Rot3Q R6 = Rot3Q::rodriguez(w6);
  Rot3Q R6m = Rot3M::rodriguez(w6);
	CHECK(assert_equal(w6, Rot3Q::Logmap(R6)));
  CHECK(assert_equal(R6m, R6));

	Vector w7 = Vector_(3, 0.0, boost::math::constants::pi<double>(), 0.0);
	Rot3Q R7 = Rot3Q::rodriguez(w7);
  Rot3Q R7m = Rot3M::rodriguez(w7);
	CHECK(assert_equal(w7, Rot3Q::Logmap(R7)));
  CHECK(assert_equal(R7m, R7));

	Vector w8 = Vector_(3, 0.0, 0.0, boost::math::constants::pi<double>());
	Rot3Q R8 = Rot3Q::rodriguez(w8);
  Rot3Q R8m = Rot3M::rodriguez(w8);
	CHECK(assert_equal(w8, Rot3Q::Logmap(R8)));
  CHECK(assert_equal(R8m, R8));
}

/* ************************************************************************* */
TEST(Rot3Q, manifold)
{
	Rot3Q gR1 = Rot3Q::rodriguez(0.1, 0.4, 0.2);
	Rot3Q gR2 = Rot3Q::rodriguez(0.3, 0.1, 0.7);
	Rot3Q origin;

	Rot3M gR1m = Rot3M::rodriguez(0.1, 0.4, 0.2);
  Rot3M gR2m = Rot3M::rodriguez(0.3, 0.1, 0.7);

	EXPECT(assert_equal(gR1m.matrix(), gR1.matrix()));
	EXPECT(assert_equal(gR2m.matrix(), gR2.matrix()));

	// log behaves correctly
	Vector d12 = gR1.localCoordinates(gR2);
	EXPECT(assert_equal(gR1m.localCoordinates(gR2m), d12));
	CHECK(assert_equal(gR2, gR1.retract(d12)));
	CHECK(assert_equal(gR2, gR1*Rot3Q::Expmap(d12)));
	Vector d21 = gR2.localCoordinates(gR1);
  EXPECT(assert_equal(gR2m.localCoordinates(gR1m), d21));
	CHECK(assert_equal(gR1, gR2.retract(d21)));
	CHECK(assert_equal(gR1, gR2*Rot3Q::Expmap(d21)));

	// Check that log(t1,t2)=-log(t2,t1)
	CHECK(assert_equal(d12,-d21));

	// lines in canonical coordinates correspond to Abelian subgroups in SO(3)
	Vector d = Vector_(3, 0.1, 0.2, 0.3);
	// exp(-d)=inverse(exp(d))
	CHECK(assert_equal(Rot3Q::Expmap(-d),Rot3Q::Expmap(d).inverse()));
	// exp(5d)=exp(2*d+3*d)=exp(2*d)exp(3*d)=exp(3*d)exp(2*d)
	Rot3Q R2 = Rot3Q::Expmap (2 * d);
	Rot3Q R3 = Rot3Q::Expmap (3 * d);
	Rot3Q R5 = Rot3Q::Expmap (5 * d);
	CHECK(assert_equal(R5,R2*R3));
	CHECK(assert_equal(R5,R3*R2));
}

/* ************************************************************************* */
class AngularVelocity: public Point3 {
public:
	AngularVelocity(const Point3& p) :
		Point3(p) {
	}
	AngularVelocity(double wx, double wy, double wz) :
		Point3(wx, wy, wz) {
	}
};

AngularVelocity bracket(const AngularVelocity& X, const AngularVelocity& Y) {
	return X.cross(Y);
}

/* ************************************************************************* */
TEST(Rot3Q, BCH)
{
	// Approximate exmap by BCH formula
	AngularVelocity w1(0.2, -0.1, 0.1);
	AngularVelocity w2(0.01, 0.02, -0.03);
	Rot3Q R1 = Rot3Q::Expmap (w1.vector()), R2 = Rot3Q::Expmap (w2.vector());
	Rot3Q R3 = R1 * R2;
	Vector expected = Rot3Q::Logmap(R3);
	Vector actual = BCH(w1, w2).vector();
	CHECK(assert_equal(expected, actual,1e-5));
}

/* ************************************************************************* */
TEST( Rot3Q, rotate_derivatives)
{
	Matrix actualDrotate1a, actualDrotate1b, actualDrotate2;
	R.rotate(P, actualDrotate1a, actualDrotate2);
	R.inverse().rotate(P, actualDrotate1b, boost::none);
	Matrix numerical1 = numericalDerivative21(testing::rotate<Rot3Q,Point3>, R, P);
	Matrix numerical2 = numericalDerivative21(testing::rotate<Rot3Q,Point3>, R.inverse(), P);
	Matrix numerical3 = numericalDerivative22(testing::rotate<Rot3Q,Point3>, R, P);
	EXPECT(assert_equal(numerical1,actualDrotate1a,error));
	EXPECT(assert_equal(numerical2,actualDrotate1b,error));
	EXPECT(assert_equal(numerical3,actualDrotate2, error));
}

/* ************************************************************************* */
TEST( Rot3Q, unrotate)
{
	Point3 w = R * P;
	Matrix H1,H2;
	Point3 actual = R.unrotate(w,H1,H2);
	CHECK(assert_equal(P,actual));

	Matrix numerical1 = numericalDerivative21(testing::unrotate<Rot3Q,Point3>, R, w);
	CHECK(assert_equal(numerical1,H1,error));

	Matrix numerical2 = numericalDerivative22(testing::unrotate<Rot3Q,Point3>, R, w);
	CHECK(assert_equal(numerical2,H2,error));
}

/* ************************************************************************* */
TEST( Rot3Q, compose )
{
	Rot3Q R1 = Rot3Q::rodriguez(0.1, 0.2, 0.3);
	Rot3Q R2 = Rot3Q::rodriguez(0.2, 0.3, 0.5);

	Rot3Q expected = R1 * R2;
	Matrix actualH1, actualH2;
	Rot3Q actual = R1.compose(R2, actualH1, actualH2);
	CHECK(assert_equal(expected,actual));

	Matrix numericalH1 = numericalDerivative21(testing::compose<Rot3Q>, R1,
			R2, 1e-2);
	CHECK(assert_equal(numericalH1,actualH1));

	Matrix numericalH2 = numericalDerivative22(testing::compose<Rot3Q>, R1,
			R2, 1e-2);
	CHECK(assert_equal(numericalH2,actualH2));
}

/* ************************************************************************* */
TEST( Rot3Q, inverse )
{
	Rot3Q R = Rot3Q::rodriguez(0.1, 0.2, 0.3);

	Rot3Q I;
	Matrix actualH;
	CHECK(assert_equal(I,R*R.inverse(actualH)));
	CHECK(assert_equal(I,R.inverse()*R));

	Matrix numericalH = numericalDerivative11(testing::inverse<Rot3Q>, R);
	CHECK(assert_equal(numericalH,actualH, 1e-4));
}

/* ************************************************************************* */
TEST( Rot3Q, between )
{
  Rot3Q r1 = Rot3Q::Rz(M_PI/3.0);
  Rot3Q r2 = Rot3Q::Rz(2.0*M_PI/3.0);

  Matrix expectedr1 = Matrix_(3,3,
      0.5, -sqrt(3.0)/2.0, 0.0,
      sqrt(3.0)/2.0, 0.5, 0.0,
      0.0, 0.0, 1.0);
  EXPECT(assert_equal(expectedr1, r1.matrix()));

	Rot3Q R = Rot3Q::rodriguez(0.1, 0.4, 0.2);
	Rot3Q origin;
	CHECK(assert_equal(R, origin.between(R)));
	CHECK(assert_equal(R.inverse(), R.between(origin)));

	Rot3Q R1 = Rot3Q::rodriguez(0.1, 0.2, 0.3);
	Rot3Q R2 = Rot3Q::rodriguez(0.2, 0.3, 0.5);

	Rot3Q expected = R1.inverse() * R2;
	Matrix actualH1, actualH2;
	Rot3Q actual = R1.between(R2, actualH1, actualH2);
	CHECK(assert_equal(expected,actual));

	Matrix numericalH1 = numericalDerivative21(testing::between<Rot3Q> , R1, R2);
	CHECK(assert_equal(numericalH1,actualH1, 1e-4));
  Matrix numericalH1M = numericalDerivative21(testing::between<Rot3M> , Rot3M(R1.matrix()), Rot3M(R2.matrix()));
  CHECK(assert_equal(numericalH1M,actualH1, 1e-4));

	Matrix numericalH2 = numericalDerivative22(testing::between<Rot3Q> , R1, R2);
	CHECK(assert_equal(numericalH2,actualH2, 1e-4));
	Matrix numericalH2M = numericalDerivative22(testing::between<Rot3M> , Rot3M(R1.matrix()), Rot3M(R2.matrix()));
	CHECK(assert_equal(numericalH2M,actualH2, 1e-4));
}

/* ************************************************************************* */
TEST( Rot3Q, xyz )
{
	double t = 0.1, st = sin(t), ct = cos(t);

	// Make sure all counterclockwise
	// Diagrams below are all from from unchanging axis

	// z
	// |   * Y=(ct,st)
	// x----y
	Rot3Q expected1(1, 0, 0, 0, ct, -st, 0, st, ct);
	CHECK(assert_equal(expected1,Rot3Q::Rx(t)));

	// x
	// |   * Z=(ct,st)
	// y----z
	Rot3Q expected2(ct, 0, st, 0, 1, 0, -st, 0, ct);
	CHECK(assert_equal(expected2,Rot3Q::Ry(t)));

	// y
	// |   X=* (ct,st)
	// z----x
	Rot3Q expected3(ct, -st, 0, st, ct, 0, 0, 0, 1);
	CHECK(assert_equal(expected3,Rot3Q::Rz(t)));

	// Check compound rotation
	Rot3Q expected = Rot3Q::Rz(0.3) * Rot3Q::Ry(0.2) * Rot3Q::Rx(0.1);
	CHECK(assert_equal(expected,Rot3Q::RzRyRx(0.1,0.2,0.3)));
}

/* ************************************************************************* */
TEST( Rot3Q, yaw_pitch_roll )
{
	double t = 0.1;

	// yaw is around z axis
	CHECK(assert_equal(Rot3Q::Rz(t),Rot3Q::yaw(t)));

	// pitch is around y axis
	CHECK(assert_equal(Rot3Q::Ry(t),Rot3Q::pitch(t)));

	// roll is around x axis
	CHECK(assert_equal(Rot3Q::Rx(t),Rot3Q::roll(t)));

	// Check compound rotation
	Rot3Q expected = Rot3Q::yaw(0.1) * Rot3Q::pitch(0.2) * Rot3Q::roll(0.3);
	CHECK(assert_equal(expected,Rot3Q::ypr(0.1,0.2,0.3)));
}

/* ************************************************************************* */
TEST( Rot3Q, RQ)
{
	// Try RQ on a pure rotation
	Matrix actualK;
	Vector actual;
	boost::tie(actualK, actual) = RQ(R.matrix());
	Vector expected = Vector_(3, 0.14715, 0.385821, 0.231671);
	CHECK(assert_equal(eye(3),actualK));
	CHECK(assert_equal(expected,actual,1e-6));

	// Try using xyz call, asserting that Rot3Q::RzRyRx(x,y,z).xyz()==[x;y;z]
	CHECK(assert_equal(expected,R.xyz(),1e-6));
	CHECK(assert_equal(Vector_(3,0.1,0.2,0.3),Rot3Q::RzRyRx(0.1,0.2,0.3).xyz()));

	// Try using ypr call, asserting that Rot3Q::ypr(y,p,r).ypr()==[y;p;r]
	CHECK(assert_equal(Vector_(3,0.1,0.2,0.3),Rot3Q::ypr(0.1,0.2,0.3).ypr()));
	CHECK(assert_equal(Vector_(3,0.3,0.2,0.1),Rot3Q::ypr(0.1,0.2,0.3).rpy()));

	// Try ypr for pure yaw-pitch-roll matrices
	CHECK(assert_equal(Vector_(3,0.1,0.0,0.0),Rot3Q::yaw (0.1).ypr()));
	CHECK(assert_equal(Vector_(3,0.0,0.1,0.0),Rot3Q::pitch(0.1).ypr()));
	CHECK(assert_equal(Vector_(3,0.0,0.0,0.1),Rot3Q::roll (0.1).ypr()));

	// Try RQ to recover calibration from 3*3 sub-block of projection matrix
	Matrix K = Matrix_(3, 3, 500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0);
	Matrix A = K * R.matrix();
	boost::tie(actualK, actual) = RQ(A);
	CHECK(assert_equal(K,actualK));
	CHECK(assert_equal(expected,actual,1e-6));
}

/* ************************************************************************* */
TEST( Rot3Q, expmapStability ) {
  Vector w = Vector_(3, 78e-9, 5e-8, 97e-7);
  double theta = w.norm();
  double theta2 = theta*theta;
  Rot3Q actualR = Rot3Q::Expmap(w);
  Matrix W = Matrix_(3,3, 0.0, -w(2), w(1),
                          w(2), 0.0, -w(0),
                          -w(1), w(0), 0.0 );
  Matrix W2 = W*W;
  Matrix Rmat = eye(3) + (1.0-theta2/6.0 + theta2*theta2/120.0
      - theta2*theta2*theta2/5040.0)*W + (0.5 - theta2/24.0 + theta2*theta2/720.0)*W2 ;
  Rot3Q expectedR( Rmat );
  CHECK(assert_equal(expectedR, actualR, 1e-10));
}

// Does not work with Quaternions
///* ************************************************************************* */
//TEST( Rot3Q, logmapStability ) {
//  Vector w = Vector_(3, 1e-8, 0.0, 0.0);
//  Rot3Q R = Rot3Q::Expmap(w);
////  double tr = R.r1().x()+R.r2().y()+R.r3().z();
////  std::cout.precision(5000);
////  std::cout << "theta: " << w.norm() << std::endl;
////  std::cout << "trace: " << tr << std::endl;
////  R.print("R = ");
//  Vector actualw = Rot3Q::Logmap(R);
//  CHECK(assert_equal(w, actualw, 1e-15));
//}

/* ************************************************************************* */
TEST(Rot3Q, quaternion) {
  // NOTE: This is also verifying the ability to convert Vector to Quaternion
  Quaternion q1(0.710997408193224, 0.360544029310185, 0.594459869568306, 0.105395217842782);
  Rot3Q R1 = Rot3Q(Matrix_(3,3,
      0.271018623057411,   0.278786459830371,   0.921318086098018,
      0.578529366719085,   0.717799701969298,  -0.387385285854279,
     -0.769319620053772,   0.637998195662053,   0.033250932803219));

  Quaternion q2(0.263360579192421, 0.571813128030932, 0.494678363680335, 0.599136268678053);
  Rot3Q R2 = Rot3Q(Matrix_(3,3,
      -0.207341903877828,   0.250149415542075,   0.945745528564780,
       0.881304914479026,  -0.371869043667957,   0.291573424846290,
       0.424630407073532,   0.893945571198514,  -0.143353873763946));

  // Check creating Rot3Q from quaternion
  EXPECT(assert_equal(R1, Rot3Q(q1)));
  EXPECT(assert_equal(R1, Rot3Q::quaternion(q1.w(), q1.x(), q1.y(), q1.z())));
  EXPECT(assert_equal(R2, Rot3Q(q2)));
  EXPECT(assert_equal(R2, Rot3Q::quaternion(q2.w(), q2.x(), q2.y(), q2.z())));

  // Check converting Rot3Q to quaterion
  EXPECT(assert_equal(Vector(R1.toQuaternion().coeffs()), Vector(q1.coeffs())));
  EXPECT(assert_equal(Vector(R2.toQuaternion().coeffs()), Vector(q2.coeffs())));

  // Check that quaternion and Rot3Q represent the same rotation
  Point3 p1(1.0, 2.0, 3.0);
  Point3 p2(8.0, 7.0, 9.0);

  Point3 expected1 = R1*p1;
  Point3 expected2 = R2*p2;

  Point3 actual1 = Point3(q1*p1.vector());
  Point3 actual2 = Point3(q2*p2.vector());

  EXPECT(assert_equal(expected1, actual1));
  EXPECT(assert_equal(expected2, actual2));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

