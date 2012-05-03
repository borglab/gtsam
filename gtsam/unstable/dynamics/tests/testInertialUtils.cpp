/**
 * @file testInertialUtils.cpp
 *
 * @brief Conversions for the utility functions from the matlab implementation
 *
 * @date Nov 28, 2011
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam2/dynamics/inertialUtils.h>

using namespace gtsam;

static const double tol = 1e-5;

/* ************************************************************************* */
TEST(testInertialUtils, RRTMbn) {
	EXPECT(assert_equal(Matrix::Identity(3,3), dynamics::RRTMbn(zero(3)), tol));
	EXPECT(assert_equal(Matrix::Identity(3,3), dynamics::RRTMbn(Rot3()), tol));
	EXPECT(assert_equal(dynamics::RRTMbn(Vector_(3, 0.3, 0.2, 0.1)), dynamics::RRTMbn(Rot3::ypr(0.1, 0.2, 0.3)), tol));
}

/* ************************************************************************* */
TEST(testInertialUtils, RRTMnb) {
	EXPECT(assert_equal(Matrix::Identity(3,3), dynamics::RRTMnb(zero(3)), tol));
	EXPECT(assert_equal(Matrix::Identity(3,3), dynamics::RRTMnb(Rot3()), tol));
	EXPECT(assert_equal(dynamics::RRTMnb(Vector_(3, 0.3, 0.2, 0.1)), dynamics::RRTMnb(Rot3::ypr(0.1, 0.2, 0.3)), tol));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
