/*
 * @file testLinearApproxFactor.cpp
 * @brief tests for dummy factor that contains a linear factor
 * @author Alex Cunningham
 */

#include <iostream>
#include <gtsam/CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/slam/planarSLAM.h>
#include <gtsam/slam/LinearApproxFactor-inl.h>

using namespace std;
using namespace gtsam;

typedef LinearApproxFactor<planarSLAM::Values,planarSLAM::PointKey> ApproxFactor;

/* ************************************************************************* */
TEST ( LinearApproxFactor, basic ) {
	Symbol key1('x', 1);
	Matrix A1 = eye(2);
	Vector b = repeat(2, 1.2);
	SharedDiagonal model = noiseModel::Unit::Create(2);
	GaussianFactor::shared_ptr lin_factor(new GaussianFactor(1, A1, b, model));
	planarSLAM::Values lin_points;
	ApproxFactor f1(lin_factor, lin_points);

	EXPECT(f1.size() == 1);
	ApproxFactor::KeyVector expKeyVec;
	expKeyVec.push_back(planarSLAM::PointKey(key1.index()));
	EXPECT(assert_equal(expKeyVec, f1.nonlinearKeys()));

	planarSLAM::Values config; // doesn't really need to have any data
	Ordering ordering;
	ordering.push_back(key1);
	GaussianFactor::shared_ptr actual = f1.linearize(config, ordering);

	// Check the linearization
	CHECK(assert_equal(*lin_factor, *actual));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
