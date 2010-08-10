/*
 * @file testLinearApproxFactor.cpp
 * @brief tests for dummy factor that contains a linear factor
 * @author Alex Cunningham
 */

#include <iostream>
#include <CppUnitLite/TestHarness.h>
#include <TestableAssertions.h> // allow assert_equal() for vectors
#include <GaussianFactor.h>
#include <planarSLAM.h>
#include <LinearApproxFactor.h>

using namespace std;
using namespace gtsam;

typedef LinearApproxFactor<planarSLAM::Config,planarSLAM::PointKey,Point2> ApproxFactor;

/* ************************************************************************* */
TEST ( LinearApproxFactor, basic ) {
	Symbol key1('x', 1);
	Matrix A1 = eye(2);
	Vector b = repeat(2, 1.2);
	SharedDiagonal model = noiseModel::Unit::Create(2);
	GaussianFactor::shared_ptr lin_factor(new GaussianFactor(key1, A1, b, model));
	planarSLAM::Config lin_points;
	ApproxFactor f1(lin_factor, lin_points);

	EXPECT(f1.size() == 1);
	ApproxFactor::KeyVector expKeyVec;
	expKeyVec.push_back(planarSLAM::PointKey(key1.index()));
	EXPECT(assert_equal(expKeyVec, f1.nonlinearKeys()));

	planarSLAM::Config config; // doesn't really need to have any data
	GaussianFactor::shared_ptr actual = f1.linearize(config);

	// Check the linearization
	CHECK(assert_equal(*lin_factor, *actual));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
