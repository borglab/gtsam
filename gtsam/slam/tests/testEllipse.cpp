/**
 * @file testCorePlanarSLAM
 * @brief Test single robot, planar case, data association (for Freiburg)
 * @author Alex Cunningham
 */

#include <iostream>
#include <boost/foreach.hpp>
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>

#include <gtsam/slam/planarSLAM.h>
#include <gtsam/slam/Ellipse.h>

using namespace std;
using namespace gtsam;

const double tol = 1e-5;

/* ************************************************************************* */
TEST( testEllipse, point_covariances ) {

	// Using a bearing/range measurement model with two observations
	double range = 0.5;
	double range_sigma = 0.1, bearing_sigma = 0.1;
	double s1 = range_sigma;
	double s2 = sin(bearing_sigma)*range;
	Matrix covariance1 = Matrix_(2,2, s1*s1, 0.0,
																		  0.0, s2*s2);
	Point2 mean(0.5, 0);
	Ellipse actEllipse1(mean, covariance1);
	EXPECT(assert_equal(mean, actEllipse1.mean(), tol));
	EXPECT(assert_equal(covariance1, actEllipse1.covariance(), tol));
	EXPECT(assert_equal(Point2( 0.1, 0.0), actEllipse1.moment1(), tol));
	EXPECT(assert_equal(Point2( 0.0, 0.05), actEllipse1.moment2(), 1e-3));

	// ellipse parameters
	Matrix covariance2 = Matrix_(2,2, 0.00666667,  0.0,
																		0.0,	0.00210191);
	Ellipse actEllipse2(mean, covariance2);
	EXPECT(assert_equal(mean, actEllipse2.mean(), tol));
	EXPECT(assert_equal(covariance2, actEllipse2.covariance(), tol));
	EXPECT(assert_equal(Point2(0.0816497, 0.0), actEllipse2.moment1(), tol));
	EXPECT(assert_equal(Point2(0.0, 0.0458466), actEllipse2.moment2(), tol));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
