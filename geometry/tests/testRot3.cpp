/**
 * @file    testRot3.cpp
 * @brief   Unit tests for Rot3 class
 * @author  Alireza Fathi
 */

#include <gtsam/CppUnitLite/TestHarness.h>
#include <boost/math/constants/constants.hpp>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>

using namespace gtsam;

Rot3 R = Rot3::rodriguez(0.1, 0.4, 0.2);
Point3 P(0.2, 0.7, -2.0);
double error = 1e-9, epsilon = 0.001;

/* ************************************************************************* */
TEST( Rot3, constructor)
{
	Rot3 expected(eye(3, 3));
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
	Rot3 actual(r1, r2, r3);
	CHECK(assert_equal(actual,expected));
}

/* ************************************************************************* */
TEST( Rot3, constructor2)
{
	Matrix R = Matrix_(3, 3, 11., 12., 13., 21., 22., 23., 31., 32., 33.);
	Rot3 actual(R);
	Rot3 expected(11, 12, 13, 21, 22, 23, 31, 32, 33);
	CHECK(assert_equal(actual,expected));
}

/* ************************************************************************* */
TEST( Rot3, constructor3)
{
	Rot3 expected(1, 2, 3, 4, 5, 6, 7, 8, 9);
	Point3 r1(1, 4, 7), r2(2, 5, 8), r3(3, 6, 9);
	CHECK(assert_equal(Rot3(r1,r2,r3),expected));
}

/* ************************************************************************* */
TEST( Rot3, transpose)
{
	Rot3 R(1, 2, 3, 4, 5, 6, 7, 8, 9);
	Point3 r1(1, 2, 3), r2(4, 5, 6), r3(7, 8, 9);
	CHECK(assert_equal(inverse(R),Rot3(r1,r2,r3)));
}

/* ************************************************************************* */
TEST( Rot3, equals)
{
	CHECK(R.equals(R));
	Rot3 zero;
	CHECK(!R.equals(zero));
}

/* ************************************************************************* */
// Notice this uses J^2 whereas fast uses w*w', and has cos(t)*I + ....
Rot3 slow_but_correct_rodriguez(const Vector& w) {
	double t = norm_2(w);
	Matrix J = skewSymmetric(w / t);
	if (t < 1e-5) return Rot3();
	Matrix R = eye(3) + sin(t) * J + (1.0 - cos(t)) * (J * J);
	return R;
}

/* ************************************************************************* */
TEST( Rot3, rodriguez)
{
	Rot3 R1 = Rot3::rodriguez(epsilon, 0, 0);
	Vector w = Vector_(3, epsilon, 0., 0.);
	Rot3 R2 = slow_but_correct_rodriguez(w);
	CHECK(assert_equal(R2,R1));
}

/* ************************************************************************* */
TEST( Rot3, rodriguez2)
{
	Vector axis = Vector_(3,0.,1.,0.); // rotation around Y
	double angle = 3.14 / 4.0;
	Rot3 actual = Rot3::rodriguez(axis, angle);
	Rot3 expected(0.707388, 0, 0.706825,
			                 0, 1,        0,
			         -0.706825, 0, 0.707388);
	CHECK(assert_equal(expected,actual,1e-5));
}

/* ************************************************************************* */
TEST( Rot3, rodriguez3)
{
	Vector w = Vector_(3, 0.1, 0.2, 0.3);
	Rot3 R1 = Rot3::rodriguez(w / norm_2(w), norm_2(w));
	Rot3 R2 = slow_but_correct_rodriguez(w);
	CHECK(assert_equal(R2,R1));
}

/* ************************************************************************* */
TEST( Rot3, rodriguez4)
{
	Vector axis = Vector_(3,0.,0.,1.); // rotation around Z
	double angle = M_PI_2;
	Rot3 actual = Rot3::rodriguez(axis, angle);
	double c=cos(angle),s=sin(angle);
	Rot3 expected(c,-s, 0,
			          s, c, 0,
			          0, 0, 1);
	CHECK(assert_equal(expected,actual,1e-5));
	CHECK(assert_equal(slow_but_correct_rodriguez(axis*angle),actual,1e-5));
}

/* ************************************************************************* */
TEST( Rot3, expmap)
{
	Vector v(3);
	fill(v.begin(), v.end(), 0);
	CHECK(assert_equal(expmap(R,v), R));
}

/* ************************************************************************* */
TEST(Rot3, log)
{
	Vector w1 = Vector_(3, 0.1, 0.0, 0.0);
	Rot3 R1 = Rot3::rodriguez(w1);
	CHECK(assert_equal(w1, logmap(R1)));

	Vector w2 = Vector_(3, 0.0, 0.1, 0.0);
	Rot3 R2 = Rot3::rodriguez(w2);
	CHECK(assert_equal(w2, logmap(R2)));

	Vector w3 = Vector_(3, 0.0, 0.0, 0.1);
	Rot3 R3 = Rot3::rodriguez(w3);
	CHECK(assert_equal(w3, logmap(R3)));

	Vector w = Vector_(3, 0.1, 0.4, 0.2);
	Rot3 R = Rot3::rodriguez(w);
	CHECK(assert_equal(w, logmap(R)));

	Vector w5 = Vector_(3, 0.0, 0.0, 0.0);
	Rot3 R5 = Rot3::rodriguez(w5);
	CHECK(assert_equal(w5, logmap(R5)));

	Vector w6 = Vector_(3, boost::math::constants::pi<double>(), 0.0, 0.0);
	Rot3 R6 = Rot3::rodriguez(w6);
	CHECK(assert_equal(w6, logmap(R6)));

	Vector w7 = Vector_(3, 0.0, boost::math::constants::pi<double>(), 0.0);
	Rot3 R7 = Rot3::rodriguez(w7);
	CHECK(assert_equal(w7, logmap(R7)));

	Vector w8 = Vector_(3, 0.0, 0.0, boost::math::constants::pi<double>());
	Rot3 R8 = Rot3::rodriguez(w8);
	CHECK(assert_equal(w8, logmap(R8)));
}

/* ************************************************************************* */
TEST(Rot3, manifold)
{
	Rot3 gR1 = Rot3::rodriguez(0.1, 0.4, 0.2);
	Rot3 gR2 = Rot3::rodriguez(0.3, 0.1, 0.7);
	Rot3 origin;

	// log behaves correctly
	Vector d12 = logmap(gR1, gR2);
	CHECK(assert_equal(gR2, expmap(gR1,d12)));
	CHECK(assert_equal(gR2, gR1*expmap<Rot3>(d12)));
	Vector d21 = logmap(gR2, gR1);
	CHECK(assert_equal(gR1, expmap(gR2,d21)));
	CHECK(assert_equal(gR1, gR2*expmap<Rot3>(d21)));

	// Check that log(t1,t2)=-log(t2,t1)
	CHECK(assert_equal(d12,-d21));

	// lines in canonical coordinates correspond to Abelian subgroups in SO(3)
	Vector d = Vector_(3, 0.1, 0.2, 0.3);
	// exp(-d)=inverse(exp(d))
	CHECK(assert_equal(expmap<Rot3>(-d),inverse(expmap<Rot3>(d))));
	// exp(5d)=exp(2*d+3*d)=exp(2*d)exp(3*d)=exp(3*d)exp(2*d)
	Rot3 R2 = expmap<Rot3> (2 * d);
	Rot3 R3 = expmap<Rot3> (3 * d);
	Rot3 R5 = expmap<Rot3> (5 * d);
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
	return AngularVelocity::cross(X, Y);
}

/* ************************************************************************* */
TEST(Rot3, BCH)
{
	// Approximate exmap by BCH formula
	AngularVelocity w1(0.2, -0.1, 0.1);
	AngularVelocity w2(0.01, 0.02, -0.03);
	Rot3 R1 = expmap<Rot3> (w1.vector()), R2 = expmap<Rot3> (w2.vector());
	Rot3 R3 = R1 * R2;
	Vector expected = logmap(R3);
	Vector actual = BCH(w1, w2).vector();
	CHECK(assert_equal(expected, actual,1e-5));
}

/* ************************************************************************* */
TEST( Rot3, rotate_derivatives)
{
	Matrix actualDrotate1a, actualDrotate1b, actualDrotate2;
	Rot3::rotate(R, P, actualDrotate1a, actualDrotate2);
	Rot3::rotate(R.inverse(), P, actualDrotate1b, boost::none);
	Matrix numerical1 = numericalDerivative21(Rot3::rotate, R, P);
	Matrix numerical2 = numericalDerivative21(Rot3::rotate, R.inverse(), P);
	Matrix numerical3 = numericalDerivative22(Rot3::rotate, R, P);
	EXPECT(assert_equal(numerical1,actualDrotate1a,error));
	EXPECT(assert_equal(numerical2,actualDrotate1b,error));
	EXPECT(assert_equal(numerical3,actualDrotate2, error));
}

/* ************************************************************************* */
TEST( Rot3, unrotate)
{
	Point3 w = R * P;
	Matrix H1,H2;
	Point3 actual = Rot3::unrotate(R,w,H1,H2);
	CHECK(assert_equal(P,actual));

	Matrix numerical1 = numericalDerivative21(Rot3::unrotate, R, w);
	CHECK(assert_equal(numerical1,H1,error));

	Matrix numerical2 = numericalDerivative22(Rot3::unrotate, R, w);
	CHECK(assert_equal(numerical2,H2,error));
}

/* ************************************************************************* */
TEST( Rot3, compose )
{
	Rot3 R1 = Rot3::rodriguez(0.1, 0.2, 0.3);
	Rot3 R2 = Rot3::rodriguez(0.2, 0.3, 0.5);

	Rot3 expected = R1 * R2;
	Matrix actualH1, actualH2;
	Rot3 actual = compose(R1, R2, actualH1, actualH2);
	CHECK(assert_equal(expected,actual));

	Matrix numericalH1 = numericalDerivative21<Rot3, Rot3, Rot3> (compose, R1,
			R2, 1e-5);
	CHECK(assert_equal(numericalH1,actualH1));

	Matrix numericalH2 = numericalDerivative22<Rot3, Rot3, Rot3> (compose, R1,
			R2, 1e-5);
	CHECK(assert_equal(numericalH2,actualH2));
}

/* ************************************************************************* */

TEST( Rot3, inverse )
{
	Rot3 R = Rot3::rodriguez(0.1, 0.2, 0.3);

	Rot3 I;
	Matrix actualH;
	CHECK(assert_equal(I,R*inverse(R, actualH)));
	CHECK(assert_equal(I,inverse(R)*R));

	Matrix numericalH = numericalDerivative11<Rot3, Rot3> (inverse, R, 1e-5);
	CHECK(assert_equal(numericalH,actualH));
}

/* ************************************************************************* */
TEST( Rot3, between )
{
	Rot3 R = Rot3::rodriguez(0.1, 0.4, 0.2);
	Rot3 origin;
	CHECK(assert_equal(R, between(origin,R)));
	CHECK(assert_equal(inverse(R), between(R,origin)));

	Rot3 R1 = Rot3::rodriguez(0.1, 0.2, 0.3);
	Rot3 R2 = Rot3::rodriguez(0.2, 0.3, 0.5);

	Rot3 expected = inverse(R1) * R2;
	Matrix actualH1, actualH2;
	Rot3 actual = between(R1, R2, actualH1, actualH2);
	CHECK(assert_equal(expected,actual));

	Matrix numericalH1 = numericalDerivative21(between<Rot3> , R1, R2, 1e-5);
	CHECK(assert_equal(numericalH1,actualH1));

	Matrix numericalH2 = numericalDerivative22(between<Rot3> , R1, R2, 1e-5);
	CHECK(assert_equal(numericalH2,actualH2));
}

/* ************************************************************************* */
TEST( Rot3, xyz )
{
	double t = 0.1, st = sin(t), ct = cos(t);

	// Make sure all counterclockwise
	// Diagrams below are all from from unchanging axis

	// z
	// |   * Y=(ct,st)
	// x----y
	Rot3 expected1(1, 0, 0, 0, ct, -st, 0, st, ct);
	CHECK(assert_equal(expected1,Rot3::Rx(t)));

	// x
	// |   * Z=(ct,st)
	// y----z
	Rot3 expected2(ct, 0, st, 0, 1, 0, -st, 0, ct);
	CHECK(assert_equal(expected2,Rot3::Ry(t)));

	// y
	// |   X=* (ct,st)
	// z----x
	Rot3 expected3(ct, -st, 0, st, ct, 0, 0, 0, 1);
	CHECK(assert_equal(expected3,Rot3::Rz(t)));

	// Check compound rotation
	Rot3 expected = Rot3::Rz(0.3) * Rot3::Ry(0.2) * Rot3::Rx(0.1);
	CHECK(assert_equal(expected,Rot3::RzRyRx(0.1,0.2,0.3)));
}

/* ************************************************************************* */
TEST( Rot3, yaw_pitch_roll )
{
	double t = 0.1;

	// yaw is around z axis
	CHECK(assert_equal(Rot3::Rz(t),Rot3::yaw(t)));

	// pitch is around y axis
	CHECK(assert_equal(Rot3::Ry(t),Rot3::pitch(t)));

	// roll is around x axis
	CHECK(assert_equal(Rot3::Rx(t),Rot3::roll(t)));

	// Check compound rotation
	Rot3 expected = Rot3::yaw(0.1) * Rot3::pitch(0.2) * Rot3::roll(0.3);
	CHECK(assert_equal(expected,Rot3::ypr(0.1,0.2,0.3)));
}

/* ************************************************************************* */
TEST( Rot3, RQ)
{
	// Try RQ on a pure rotation
	Matrix actualK;
	Vector actual;
	boost::tie(actualK, actual) = RQ(R.matrix());
	Vector expected = Vector_(3, 0.14715, 0.385821, 0.231671);
	CHECK(assert_equal(eye(3),actualK));
	CHECK(assert_equal(expected,actual,1e-6));

	// Try using xyz call, asserting that Rot3::RzRyRx(x,y,z).xyz()==[x;y;z]
	CHECK(assert_equal(expected,R.xyz(),1e-6));
	CHECK(assert_equal(Vector_(3,0.1,0.2,0.3),Rot3::RzRyRx(0.1,0.2,0.3).xyz()));

	// Try using ypr call, asserting that Rot3::ypr(y,p,r).ypr()==[y;p;r]
	CHECK(assert_equal(Vector_(3,0.1,0.2,0.3),Rot3::ypr(0.1,0.2,0.3).ypr()));

	// Try ypr for pure yaw-pitch-roll matrices
	CHECK(assert_equal(Vector_(3,0.1,0.0,0.0),Rot3::yaw (0.1).ypr()));
	CHECK(assert_equal(Vector_(3,0.0,0.1,0.0),Rot3::pitch(0.1).ypr()));
	CHECK(assert_equal(Vector_(3,0.0,0.0,0.1),Rot3::roll (0.1).ypr()));

	// Try RQ to recover calibration from 3*3 sub-block of projection matrix
	Matrix K = Matrix_(3, 3, 500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0);
	Matrix A = K * R.matrix();
	boost::tie(actualK, actual) = RQ(A);
	CHECK(assert_equal(K,actualK));
	CHECK(assert_equal(expected,actual,1e-6));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

