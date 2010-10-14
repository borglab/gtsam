/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
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
	Symbol key1('l', 1);
	Matrix A1 = eye(2);
	Vector b = repeat(2, 1.2);
	SharedDiagonal model = noiseModel::Unit::Create(2);
	GaussianFactor::shared_ptr lin_factor(new GaussianFactor(0, A1, b, model));
	Ordering ordering;
	ordering.push_back(key1);
	planarSLAM::Values lin_points;
	planarSLAM::PointKey PKey(1);
	Point2 point(1.0, 2.0);
	lin_points.insert(PKey, point);
	ApproxFactor f1(lin_factor, ordering, lin_points);

	EXPECT(f1.size() == 1);
	EXPECT(assert_equal(key1, f1.keys().front()));
	EXPECT(assert_equal(b, f1.get_b()));

	planarSLAM::Values config;
	config.insert(PKey, Point2(2.0, 3.0));
	GaussianFactor::shared_ptr actual = f1.linearize(config, ordering);

	EXPECT(assert_equal(Vector_(2, -0.2, -0.2), f1.unwhitenedError(config)));

	// Check the linearization
	EXPECT(assert_equal(*lin_factor, *actual));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
