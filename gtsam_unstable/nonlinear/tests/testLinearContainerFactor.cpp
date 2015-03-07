/**
 * @file testLinearContainerFactor.cpp
 *
 * @date Jul 6, 2012
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam_unstable/nonlinear/LinearContainerFactor.h>

#include <gtsam/base/TestableAssertions.h>
#include <gtsam/geometry/Pose2.h>

using namespace gtsam;

const gtsam::noiseModel::Diagonal::shared_ptr diag_model2 = noiseModel::Diagonal::Sigmas(Vector_(2, 1.0, 1.0));
const double tol = 1e-5;

gtsam::Key	l1 = 101, l2 = 102, x1 = 1, x2 = 2;

Point2 landmark1(5.0, 1.5), landmark2(7.0, 1.5);
Pose2 poseA1(0.0, 0.0, 0.0), poseA2(2.0, 0.0, 0.0);

/* ************************************************************************* */
TEST( testLinearContainerFactor, generic_jacobian_factor ) {

	Ordering initOrdering; initOrdering += x1, x2, l1, l2;

	JacobianFactor expLinFactor1(
			initOrdering[l1],
			Matrix_(2,2,
					 2.74222, -0.0067457,
							 0.0,  2.63624),
			initOrdering[l2],
			Matrix_(2,2,
					-0.0455167, -0.0443573,
					-0.0222154, -0.102489),
			Vector_(2, 0.0277052,
								 -0.0533393),
			diag_model2);

	LinearContainerFactor actFactor1(expLinFactor1, initOrdering);
	Values values;
	values.insert(l1, landmark1);
	values.insert(l2, landmark2);
	values.insert(x1, poseA1);
	values.insert(x2, poseA2);

	// Check reconstruction from same ordering
	GaussianFactor::shared_ptr actLinearizationA = actFactor1.linearize(values, initOrdering);
	EXPECT(assert_equal(*expLinFactor1.clone(), *actLinearizationA, tol));

	// Check reconstruction from new ordering
	Ordering newOrdering; newOrdering += x1, l1, x2, l2;
	GaussianFactor::shared_ptr actLinearizationB = actFactor1.linearize(values, newOrdering);

	JacobianFactor expLinFactor2(
			newOrdering[l1],
			Matrix_(2,2,
					 2.74222, -0.0067457,
							 0.0,  2.63624),
			newOrdering[l2],
			Matrix_(2,2,
					-0.0455167, -0.0443573,
					-0.0222154, -0.102489),
			Vector_(2, 0.0277052,
								 -0.0533393),
			diag_model2);

	EXPECT(assert_equal(*expLinFactor2.clone(), *actLinearizationB, tol));
}

/* ************************************************************************* */
TEST( testLinearContainerFactor, jacobian_factor_withlinpoints ) {

	Ordering ordering; ordering += x1, x2, l1, l2;

	JacobianFactor expLinFactor(
			ordering[l1],
			Matrix_(2,2,
					 2.74222, -0.0067457,
							 0.0,  2.63624),
			ordering[l2],
			Matrix_(2,2,
					-0.0455167, -0.0443573,
					-0.0222154, -0.102489),
			Vector_(2, 0.0277052,
								 -0.0533393),
			diag_model2);

	Values values;
	values.insert(l1, landmark1);
	values.insert(l2, landmark2);
	values.insert(x1, poseA1);
	values.insert(x2, poseA2);

	LinearContainerFactor actFactor(expLinFactor, ordering, values);
	LinearContainerFactor actFactorNolin(expLinFactor, ordering);

	EXPECT(assert_equal(actFactor, actFactor, tol));
	EXPECT(assert_inequal(actFactor, actFactorNolin, tol));
	EXPECT(assert_inequal(actFactorNolin, actFactor, tol));

	// Check contents
	Values expLinPoint;
	expLinPoint.insert(l1, landmark1);
	expLinPoint.insert(l2, landmark2);
	CHECK(actFactor.linearizationPoint());
	EXPECT(assert_equal(expLinPoint, *actFactor.linearizationPoint()));

	// Check error evaluation
	VectorValues delta = values.zeroVectors(ordering);
	delta.at(ordering[l1]) = Vector_(2, 1.0, 2.0);
	delta.at(ordering[l2]) = Vector_(2, 3.0, 4.0);
	Values noisyValues = values.retract(delta, ordering);
	double expError = expLinFactor.error(delta);
	EXPECT_DOUBLES_EQUAL(expError, actFactor.error(noisyValues), tol);
	EXPECT_DOUBLES_EQUAL(expLinFactor.error(values.zeroVectors(ordering)), actFactor.error(values), tol);
}

/* ************************************************************************* */
TEST( testLinearContainerFactor, generic_hessian_factor ) {
  Matrix G11 = Matrix_(1,1, 1.0);
  Matrix G12 = Matrix_(1,2, 2.0, 4.0);
  Matrix G13 = Matrix_(1,3, 3.0, 6.0, 9.0);

  Matrix G22 = Matrix_(2,2, 3.0, 5.0, 0.0, 6.0);
  Matrix G23 = Matrix_(2,3, 4.0, 6.0, 8.0, 1.0, 2.0, 4.0);

  Matrix G33 = Matrix_(3,3, 1.0, 2.0, 3.0, 0.0, 5.0, 6.0, 0.0, 0.0, 9.0);

  Vector g1 = Vector_(1, -7.0);
  Vector g2 = Vector_(2, -8.0, -9.0);
  Vector g3 = Vector_(3,  1.0,  2.0,  3.0);

  double f = 10.0;

	Ordering initOrdering; initOrdering += x1, x2, l1, l2;
  HessianFactor initFactor(initOrdering[x1], initOrdering[x2], initOrdering[l1],
  		G11, G12, G13, g1, G22, G23, g2, G33, g3, f);

	Values values;
	values.insert(l1, landmark1);
	values.insert(l2, landmark2);
	values.insert(x1, poseA1);
	values.insert(x2, poseA2);

  LinearContainerFactor actFactor(initFactor, initOrdering);
	GaussianFactor::shared_ptr actLinearization1 = actFactor.linearize(values, initOrdering);
	EXPECT(assert_equal(*initFactor.clone(), *actLinearization1, tol));

	Ordering newOrdering; newOrdering += l1, x1, x2, l2;
  HessianFactor expLinFactor(newOrdering[x1], newOrdering[x2], newOrdering[l1],
  		G11, G12, G13, g1, G22, G23, g2, G33, g3, f);
  GaussianFactor::shared_ptr actLinearization2 = actFactor.linearize(values, newOrdering);
 	EXPECT(assert_equal(*expLinFactor.clone(), *actLinearization2, tol));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
