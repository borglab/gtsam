/**
 * @file testLieVector.cpp
 * @author Alex Cunningham
 */

#include <gtsam/CppUnitLite/TestHarness.h>

#include <gtsam/base/LieVector.h>

using namespace gtsam;

/* ************************************************************************* */
TEST( testLieVector, construction ) {
	Vector v = Vector_(3, 1.0, 2.0, 3.0);
	LieVector lie1(v), lie2(v);

	EXPECT(lie1.dim() == 3);
	EXPECT(assert_equal(v, lie1.vector()));
	EXPECT(assert_equal(lie1, lie2));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */


