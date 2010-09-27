/**
 * @file   testPoint2.cpp
 * @brief  Unit tests for Point2 class
 * @author Frank Dellaert
 **/

#include <gtsam/CppUnitLite/TestHarness.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point2.h>

using namespace std;
using namespace gtsam;

const double tol = 1e-5;

/* ************************************************************************* */
TEST( Point2, expmap)
{
	Vector d(2);
	d(0) = 1;
	d(1) = -1;
	Point2 a(4, 5), b = a.expmap(d), c(5, 4);
	CHECK(assert_equal(b,c));
}

/* ************************************************************************* */
TEST( Point2, arithmetic)
{
	CHECK(assert_equal( Point2(-5,-6), -Point2(5,6) ));
	CHECK(assert_equal( Point2(5,6), Point2(4,5)+Point2(1,1)));
	CHECK(assert_equal( Point2(3,4), Point2(4,5)-Point2(1,1) ));
	CHECK(assert_equal( Point2(8,6), Point2(4,3)*2));
	CHECK(assert_equal( Point2(4,6), 2*Point2(2,3)));
	CHECK(assert_equal( Point2(2,3), Point2(4,6)/2));
}

/* ************************************************************************* */
TEST( Point2, norm)
{
	Point2 p0(cos(5), sin(5));
	DOUBLES_EQUAL(1,p0.norm(),1e-6);
	Point2 p1(4, 5), p2(1, 1);
	DOUBLES_EQUAL( 5,p1.dist(p2),1e-6);
	DOUBLES_EQUAL( 5,(p2-p1).norm(),1e-6);
}

/* ************************************************************************* */
Point2 transform_from_proxy(const Point2& pose, const Point2& point) {
	return pose.transform_from(point);
}

/* ************************************************************************* */
Point2 transform_to_proxy(const Point2& pose, const Point2& point) {
	return pose.transform_to(point);
}

Point2 offset(3.0, 4.0), pt(-5.0, 6.0);

/* ************************************************************************* */
TEST( Point2, transforms ) {
	EXPECT(assert_equal(Point2(5.0, 6.0), offset.transform_from(Point2(2.0, 2.0))));
	EXPECT(assert_equal(Point2(-1.0, -2.0), offset.transform_to(Point2(2.0, 2.0))));
	EXPECT(assert_equal(Point2(1.0, 2.0), offset.transform_to(
			offset.transform_from(Point2(1.0, 2.0)))));
}

/* ************************************************************************* */
TEST( Point2, transform_to_derivatives ) {
	Matrix actH1, actH2;
	offset.transform_to(pt, actH1, actH2);
	Matrix numericalH1 = numericalDerivative21(transform_to_proxy, offset, pt, 1e-5);
	Matrix numericalH2 = numericalDerivative22(transform_to_proxy, offset, pt, 1e-5);
	EXPECT(assert_equal(numericalH1, actH1, tol));
	EXPECT(assert_equal(numericalH2, actH2, tol));
}

/* ************************************************************************* */
TEST( Point2, transform_from_derivatives ) {
	Matrix actH1, actH2;
	offset.transform_from(pt, actH1, actH2);
	Matrix numericalH1 = numericalDerivative21(transform_from_proxy, offset, pt, 1e-5);
	Matrix numericalH2 = numericalDerivative22(transform_from_proxy, offset, pt, 1e-5);
	EXPECT(assert_equal(numericalH1, actH1, tol));
	EXPECT(assert_equal(numericalH2, actH2, tol));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

