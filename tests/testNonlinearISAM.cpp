/**
 * @file testNonlinearISAM
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam/linear/Sampler.h>
#include <gtsam/nonlinear/NonlinearISAM-inl.h>
#include <gtsam/slam/planarSLAM.h>

using namespace gtsam;
using namespace planarSLAM;

typedef NonlinearISAM<Values> PlanarISAM;

const double tol=1e-5;

/* ************************************************************************* */
TEST(testNonlinearISAM, markov_chain ) {
	int reorder_interval = 2;
	PlanarISAM isam(reorder_interval);

	SharedDiagonal model = noiseModel::Diagonal::Sigmas(Vector_(3, 3.0, 3.0, 0.5));
	Sampler sampler(model, 42u);

	// create initial graph
	PoseKey key(0);
	Pose2 cur_pose; // start at origin
	Graph start_factors;
	start_factors.addPoseConstraint(key, cur_pose);
	Values init;
	Values expected;
	init.insert(key, cur_pose);
	expected.insert(key, cur_pose);
	isam.update(start_factors, init);

	// loop for a period of time to verify memory usage
	size_t nrPoses = 21;
	Pose2 z(1.0, 2.0, 0.1);
	for (size_t i=1; i<=nrPoses; ++i) {
		Graph new_factors;
		PoseKey key1(i-1), key2(i);
		new_factors.addOdometry(key1, key2, z, model);
		Values new_init;
		cur_pose = cur_pose.compose(z);
		new_init.insert(key2, cur_pose.expmap(sampler.sample()));
		expected.insert(key2, cur_pose);
		isam.update(new_factors, new_init);
	}

	// verify values - all but the last one should be very close
	Values actual = isam.estimate();
	for (size_t i=0; i<21; ++i) {
		PoseKey cur_key(i);
		EXPECT(assert_equal(expected[cur_key], actual[cur_key], tol));
	}
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
