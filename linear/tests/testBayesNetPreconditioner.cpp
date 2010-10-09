/**
 *  @file   testBayesNetConditioner.cpp
 *  @brief  Unit tests for BayesNetConditioner
 *  @author Frank Dellaert
 **/

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <gtsam/CppUnitLite/TestHarness.h>

#define GTSAM_MAGIC_KEY

#include <gtsam/inference/Ordering.h>
#include <gtsam/linear/BayesNetPreconditioner.h>
#include <gtsam/linear/iterative-inl.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( BayesNetPreconditioner, operators )
{
	// Build a simple Bayes net
	// small Bayes Net x <- y, x=2D, y=1D
	// 1 2 3   x1   0
	// 0 1 2 * x2 = 0
	// 0 0 1   x3   1

	// Create a scalar Gaussian on y
	GaussianBayesNet bn = scalarGaussian("y", 1, 0.1);

	// Add a conditional node with one parent |Rx+Sy-d|
	Matrix R11 = Matrix_(2, 2, 1.0, 2.0, 0.0, 1.0), S12 = Matrix_(2, 1, 3.0, 2.0);
	Vector d = zero(2);
	Vector sigmas = Vector_(2, 0.1, 0.1);
	push_front(bn, "x", d, R11, "y", S12, sigmas);

	// Create Precondioner class
	GaussianFactorGraph dummy;
	BayesNetPreconditioner P(dummy,bn);

	// inv(R1)*d should equal solution [1;-2;1]
	VectorValues D;
	D.insert("x", d);
	D.insert("y", Vector_(1, 1.0 / 0.1)); // corrected by sigma
	VectorValues expected1;
	expected1.insert("x", Vector_(2, 1.0, -2.0));
	expected1.insert("y", Vector_(1, 1.0));
	VectorValues actual1 = P.backSubstitute(D);
	CHECK(assert_equal(expected1,actual1));

	// inv(R1')*ones should equal ?
	VectorValues ones;
	ones.insert("x", Vector_(2, 1.0, 1.0));
	ones.insert("y", Vector_(1, 1.0));
	VectorValues expected2;
	expected2.insert("x", Vector_(2, 0.1, -0.1));
	expected2.insert("y", Vector_(1, 0.0));
	VectorValues actual2 = P.backSubstituteTranspose(ones);
	CHECK(assert_equal(expected2,actual2));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
