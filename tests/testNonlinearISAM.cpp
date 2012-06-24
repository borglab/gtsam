/**
 * @file testNonlinearISAM
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam/linear/Sampler.h>
#include <gtsam/nonlinear/NonlinearISAM.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/slam/planarSLAM.h>

using namespace gtsam;
using namespace planarSLAM;

typedef NonlinearISAM PlanarISAM;

const double tol=1e-5;

/* ************************************************************************* */
TEST(testNonlinearISAM, markov_chain ) {
	int reorder_interval = 2;
	PlanarISAM isam(reorder_interval); // create an ISAM object

	SharedDiagonal model = noiseModel::Diagonal::Sigmas(Vector_(3, 3.0, 3.0, 0.5));
	Sampler sampler(model, 42u);

	// create initial graph
	Pose2 cur_pose; // start at origin
	Graph start_factors;
	start_factors.addPoseConstraint(0, cur_pose);

	planarSLAM::Values init;
	planarSLAM::Values expected;
	init.insertPose(0, cur_pose);
	expected.insertPose(0, cur_pose);
	isam.update(start_factors, init);

	// loop for a period of time to verify memory usage
	size_t nrPoses = 21;
	Pose2 z(1.0, 2.0, 0.1);
	for (size_t i=1; i<=nrPoses; ++i) {
		Graph new_factors;
		new_factors.addRelativePose(i-1, i, z, model);
		planarSLAM::Values new_init;

		// perform a check on changing orderings
		if (i == 5) {
			Ordering ordering = isam.getOrdering();

			// swap last two elements
			Key last = ordering.pop_back().first;
			Key secondLast = ordering.pop_back().first;
			ordering.push_back(last);
			ordering.push_back(secondLast);
			isam.setOrdering(ordering);

			Ordering expected; expected += (0), (1), (2), (4), (3);
			EXPECT(assert_equal(expected, isam.getOrdering()));
		}

		cur_pose = cur_pose.compose(z);
		new_init.insertPose(i, cur_pose.retract(sampler.sample()));
		expected.insertPose(i, cur_pose);
		isam.update(new_factors, new_init);
	}

	// verify values - all but the last one should be very close
	planarSLAM::Values actual = isam.estimate();
	for (size_t i=0; i<nrPoses; ++i) {
		EXPECT(assert_equal(expected.pose(i), actual.pose(i), tol));
	}
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
