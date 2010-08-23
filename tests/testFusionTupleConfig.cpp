/**
 * @file testFusionTupleConfig.cpp
 * @author Alex Cunningham
 */

#include <iostream>

#include <gtsam/CppUnitLite/TestHarness.h>

#define GTSAM_MAGIC_KEY

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/VectorConfig.h>

#include <gtsam/slam/BearingRangeFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtsam/nonlinear/NonlinearFactorGraph-inl.h>
#include <gtsam/nonlinear/LieConfig-inl.h>
#include <gtsam/nonlinear/NonlinearOptimizer-inl.h>
#include <gtsam/nonlinear/FusionTupleConfig.h>

using namespace boost;
using namespace gtsam;
using namespace std;

static const double tol = 1e-5;

typedef TypedSymbol<Pose2, 'x'> PoseKey;
typedef TypedSymbol<Point2, 'l'> PointKey;
typedef LieConfig<PoseKey> PoseConfig;
typedef LieConfig<PointKey> PointConfig;

// some generic poses, points and keys
PoseKey x1(1), x2(2);
Pose2 x1_val(1.0, 2.0, 0.3), x2_val(2.0, 3.0, 0.4);
PointKey l1(1), l2(2);
Point2 l1_val(1.0, 2.0), l2_val(3.0, 4.0);

typedef FusionTupleConfig<fusion::set<PointConfig> > TupPointConfig;
typedef FusionTupleConfig<fusion::set<PoseConfig> > TupPoseConfig;
typedef FusionTupleConfig<fusion::set<PoseConfig, PointConfig> > TupPairConfig;

/* ************************************************************************* */
TEST( testFusionTupleConfig, basic_config_operations ) {

	// some initialized configs to start with
	PoseConfig poseConfig1;
	poseConfig1.insert(x1, x1_val);
	PointConfig pointConfig1;
	pointConfig1.insert(l1, l1_val);

	// basic initialization
	TupPointConfig cfg1A, cfg2A(pointConfig1);
	EXPECT(cfg1A.size() == 0);
	EXPECT(cfg2A.size() == 1);
	EXPECT(cfg1A.empty());
	EXPECT(!cfg2A.empty());

	TupPoseConfig cfg1B, cfg2B(poseConfig1);
	EXPECT(cfg1B.size() == 0);
	EXPECT(cfg2B.size() == 1);
	EXPECT(cfg1B.empty());
	EXPECT(!cfg2B.empty());

	TupPairConfig cfg1, cfg2(fusion::make_set(poseConfig1, pointConfig1));
	EXPECT(cfg1.size() == 0);
	EXPECT(cfg1.empty());
	EXPECT(cfg2.size() == 2);
	EXPECT(!cfg2.empty());

	// insertion
	cfg1A.insert(l1, l1_val);
	EXPECT(cfg1A.size() == 1);
	cfg1A.insert(l2, l2_val);
	EXPECT(cfg1A.size() == 2);
	cfg2A.insert(l2, l2_val);
	EXPECT(cfg2A.size() == 2);

	cfg1B.insert(x1, x1_val);
	EXPECT(cfg1B.size() == 1);
	cfg1B.insert(x2, x2_val);
	EXPECT(cfg1B.size() == 2);
	cfg2B.insert(x2, x2_val);
	EXPECT(cfg2B.size() == 2);

	cfg1.insert(x1, x1_val);
	cfg1.insert(x2, x2_val);
	cfg1.insert(l1, l1_val);
	cfg1.insert(l2, l2_val);
	EXPECT(cfg1.size() == 4);

	// exists
	EXPECT(cfg1.exists(x1));
	EXPECT(cfg1.exists(x2));
	EXPECT(cfg1.exists(l1));
	EXPECT(cfg1.exists(l2));

	// retrieval of elements
	EXPECT(assert_equal(l1_val, cfg1A.at(l1)));
	EXPECT(assert_equal(l2_val, cfg1A.at(l2)));

	EXPECT(assert_equal(x1_val, cfg1B.at(x1)));
	EXPECT(assert_equal(x2_val, cfg1B.at(x2)));

	EXPECT(assert_equal(l1_val, cfg1.at(l1)));
	EXPECT(assert_equal(l2_val, cfg1.at(l2)));
	EXPECT(assert_equal(x1_val, cfg1.at(x1)));
	EXPECT(assert_equal(x2_val, cfg1.at(x2)));

	// config extraction
	PointConfig expPointConfig;
	expPointConfig.insert(l1, l1_val);
	expPointConfig.insert(l2, l2_val);

	PoseConfig expPoseConfig;
	expPoseConfig.insert(x1, x1_val);
	expPoseConfig.insert(x2, x2_val);

	EXPECT(assert_equal(expPointConfig, cfg1A.config<PointConfig>()));
	EXPECT(assert_equal(expPoseConfig, cfg1B.config<PoseConfig>()));

	EXPECT(assert_equal(expPointConfig, cfg1.config<PointConfig>()));
	EXPECT(assert_equal(expPoseConfig, cfg1.config<PoseConfig>()));

	// getting sizes of configs
	EXPECT(cfg1A.nrConfigs() == 1);
	EXPECT(cfg1B.nrConfigs() == 1);
	EXPECT(cfg1.nrConfigs() == 2);

	// erase
	cfg1.erase(x1);
	EXPECT(cfg1.size() == 3);
	EXPECT(!cfg1.exists(x1));
	cfg1.erase(l1);
	EXPECT(cfg1.size() == 2);
	EXPECT(!cfg1.exists(l1));

	// clear
	cfg1.clear();
	cfg1A.clear(); cfg1B.clear();
	EXPECT(cfg1.size() == 0);
	EXPECT(cfg1.empty());
	EXPECT(cfg1A.size() == 0);
	EXPECT(cfg1A.empty());
	EXPECT(cfg1B.size() == 0);
	EXPECT(cfg1B.empty());

	cfg2.clear();
	cfg2A.clear(); cfg2B.clear();
	EXPECT(cfg2.size() == 0);
	EXPECT(cfg2.empty());
	EXPECT(cfg2A.size() == 0);
	EXPECT(cfg2A.empty());
	EXPECT(cfg2B.size() == 0);
	EXPECT(cfg2B.empty());

}

/* ************************************************************************* */
TEST( testFusionTupleConfig, slicing ) {
	TupPairConfig cfg_pair;
	cfg_pair.insert(x1, x1_val);
	cfg_pair.insert(x2, x2_val);
	cfg_pair.insert(l1, l1_val);
	cfg_pair.insert(l2, l2_val);

	// initialize configs by slicing a larger config
	TupPoseConfig cfg_pose(cfg_pair);
	TupPointConfig cfg_point(cfg_pair);

	PointConfig expPointConfig;
	expPointConfig.insert(l1, l1_val);
	expPointConfig.insert(l2, l2_val);

	PoseConfig expPoseConfig;
	expPoseConfig.insert(x1, x1_val);
	expPoseConfig.insert(x2, x2_val);

	// verify
	EXPECT(assert_equal(expPointConfig, cfg_point.config<PointConfig>()));
	EXPECT(assert_equal(expPoseConfig, cfg_pose.config<PoseConfig>()));
}

/* ************************************************************************* */
TEST( testFusionTupleConfig, upgrading ) {
	TupPoseConfig small;
	small.insert(x1, x1_val);

	TupPairConfig actual(small);
	EXPECT(actual.size() == 1);
	EXPECT(assert_equal(x1_val, actual.at(x1), tol));
}

/* ************************************************************************* */
TEST( testFusionTupleConfig, equals ) {
	TupPairConfig c1, c2, c3, c4, c5, c6;
	c1.insert(l1, l1_val);
	c1.insert(l2, l2_val);
	c1.insert(x1, x1_val);
	c1.insert(x2, x2_val);
	c4 = c1;
	c2.insert(x1, x1_val);
	c2.insert(x2, x2_val);

	EXPECT(assert_equal(c1, c1, tol));
	EXPECT(assert_equal(c4, c1, tol));
	EXPECT(assert_equal(c4, c4, tol));
	EXPECT(!c1.equals(c2, tol));
	EXPECT(!c2.equals(c1, tol));
	EXPECT(!c1.equals(c3, tol));
	EXPECT(!c2.equals(c3, tol));
	EXPECT(assert_equal(c3, c3));

	c5.insert(l1, l1_val);
	c6.insert(l1, l1_val + Point2(1e-6, 1e-6));
	EXPECT(assert_equal(c5, c6, 1e-5));
}

/* ************************************************************************* */
TEST( testFusionTupleConfig, insert_equals1 )
{
	TupPairConfig expected;
	expected.insert(PoseKey(1), x1_val);
	expected.insert(PoseKey(2), x2_val);
	expected.insert(PointKey(1), l1_val);
	expected.insert(PointKey(2), l2_val);

	TupPairConfig actual;
	actual.insert(PoseKey(1), x1_val);
	actual.insert(PoseKey(2), x2_val);
	actual.insert(PointKey(1), l1_val);
	actual.insert(PointKey(2), l2_val);

	CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( testFusionTupleConfig, insert_equals2 )
{
	TupPairConfig config1;
	config1.insert(PoseKey(1), x1_val);
	config1.insert(PoseKey(2), x2_val);
	config1.insert(PointKey(1), l1_val);
	config1.insert(PointKey(2), l2_val);

	TupPairConfig config2;
	config2.insert(PoseKey(1), x1_val);
	config2.insert(PoseKey(2), x2_val);
	config2.insert(PointKey(1), l1_val);

	EXPECT(!config1.equals(config2));

	config2.insert(PointKey(2), Point2(9,11));

	EXPECT(!config1.equals(config2));
}

/* ************************************************************************* */
TEST( testFusionTupleConfig, insert_duplicate )
{
	TupPairConfig config1;
	config1.insert(x1, x1_val); // 3
	config1.insert(x2, x2_val); // 6
	config1.insert(l1, l1_val); // 8
	config1.insert(l2, l2_val); // 10
	config1.insert(l2, l1_val); // still 10 !!!!

	EXPECT(assert_equal(l2_val, config1[PointKey(2)]));
	LONGS_EQUAL(4,config1.size());
	LONGS_EQUAL(10,config1.dim());
}

/* ************************************************************************* */
TEST( testFusionTupleConfig, size_dim )
{
	TupPairConfig config1;
	config1.insert(PoseKey(1), x1_val);
	config1.insert(PoseKey(2), x2_val);
	config1.insert(PointKey(1), l1_val);
	config1.insert(PointKey(2), l2_val);

	EXPECT(config1.size() == 4);
	EXPECT(config1.dim() == 10);
}

/* ************************************************************************* */
TEST( testFusionTupleConfig, at)
{
	TupPairConfig config1;
	config1.insert(PoseKey(1), x1_val);
	config1.insert(PoseKey(2), x2_val);
	config1.insert(PointKey(1), l1_val);
	config1.insert(PointKey(2), l2_val);

	EXPECT(assert_equal(x1_val, config1[PoseKey(1)]));
	EXPECT(assert_equal(x2_val, config1[PoseKey(2)]));
	EXPECT(assert_equal(l1_val, config1[PointKey(1)]));
	EXPECT(assert_equal(l2_val, config1[PointKey(2)]));

	CHECK_EXCEPTION(config1[PoseKey(3)], std::invalid_argument);
	CHECK_EXCEPTION(config1[PointKey(3)], std::invalid_argument);
}

/* ************************************************************************* */
TEST( testFusionTupleConfig, zero_expmap_logmap)
{
	Pose2 xA1(1,2,3), xA2(6,7,8);
	Point2 lA1(4,5), lA2(9,10);

	TupPairConfig config1;
	config1.insert(PoseKey(1), xA1);
	config1.insert(PoseKey(2), xA2);
	config1.insert(PointKey(1), lA1);
	config1.insert(PointKey(2), lA2);

	VectorConfig expected_zero;
	expected_zero.insert("x1", zero(3));
	expected_zero.insert("x2", zero(3));
	expected_zero.insert("l1", zero(2));
	expected_zero.insert("l2", zero(2));

	EXPECT(assert_equal(expected_zero, config1.zero()));

	VectorConfig delta;
	delta.insert("x1", Vector_(3, 1.0, 1.1, 1.2));
	delta.insert("x2", Vector_(3, 1.3, 1.4, 1.5));
	delta.insert("l1", Vector_(2, 1.0, 1.1));
	delta.insert("l2", Vector_(2, 1.3, 1.4));

	TupPairConfig expected;
	expected.insert(PoseKey(1), expmap(xA1, Vector_(3, 1.0, 1.1, 1.2)));
	expected.insert(PoseKey(2), expmap(xA2, Vector_(3, 1.3, 1.4, 1.5)));
	expected.insert(PointKey(1), Point2(5.0, 6.1));
	expected.insert(PointKey(2), Point2(10.3, 11.4));

	TupPairConfig actual = expmap(config1, delta);
	EXPECT(assert_equal(expected, actual));

	// Check log
	VectorConfig expected_log = delta;
	VectorConfig actual_log = logmap(config1,actual);
	EXPECT(assert_equal(expected_log, actual_log));
}

/* ************************************************************************* */
TEST( testFusionTupleConfig, partial_insert) {
	TupPairConfig init, expected;

	Pose2 pose1(1.0, 2.0, 0.3), pose2(3.0, 4.0, 5.0);
	Point2 point1(2.0, 3.0), point2(5.0, 6.0);

	init.insert(x1, pose1);
	init.insert(l1, point1);

	PoseConfig cfg1;
	cfg1.insert(x2, pose2);

	init.insertSub(cfg1);

	expected.insert(x1, pose1);
	expected.insert(l1, point1);
	expected.insert(x2, pose2);

	CHECK(assert_equal(expected, init));
}

/* ************************************************************************* */
TEST( testFusionTupleConfig, update) {
	TupPairConfig init, superset, expected;

	Pose2 pose1(1.0, 2.0, 0.3), pose2(3.0, 4.0, 5.0);
	Point2 point1(2.0, 3.0), point2(5.0, 6.0);

	init.insert(x1, pose1);
	init.insert(l1, point1);

	Pose2 pose1_(1.0, 2.0, 0.4);
	Point2 point1_(2.0, 4.0);
	superset.insert(x1, pose1_);
	superset.insert(l1, point1_);
	superset.insert(x2, pose2);
	superset.insert(l2, point2);
	init.update(superset);

	expected.insert(x1, pose1_);
	expected.insert(l1, point1_);

	CHECK(assert_equal(expected, init));
}

/* ************************************************************************* */
TEST( testFusionTupleConfig, update_element )
{
	TupPairConfig cfg;
	PoseKey xk(1);
	PointKey lk(1);

	cfg.insert(xk, x1_val);
	EXPECT(cfg.size() == 1);
	EXPECT(assert_equal(x1_val, cfg.at(xk)));

	cfg.update(xk, x2_val);
	EXPECT(cfg.size() == 1);
	EXPECT(assert_equal(x2_val, cfg.at(xk)));

	cfg.insert(lk, l1_val);
	EXPECT(cfg.size() == 2);
	EXPECT(assert_equal(l1_val, cfg.at(lk)));

	cfg.update(lk, l2_val);
	EXPECT(cfg.size() == 2);
	EXPECT(assert_equal(l2_val, cfg.at(lk)));
}

/* ************************************************************************* */
TEST( testFusionTupleConfig, expmap)
{
	Pose2 xA1(1,2,3), xA2(6,7,8);
	PoseKey x1k(1), x2k(2);
	Point2 lA1(4,5), lA2(9,10);
	PointKey l1k(1), l2k(2);

	TupPairConfig config1;
	config1.insert(x1k, xA1);
	config1.insert(x2k, xA2);
	config1.insert(l1k, lA1);
	config1.insert(l2k, lA2);

	VectorConfig delta;
	delta.insert("x1", Vector_(3, 1.0, 1.1, 1.2));
	delta.insert("x2", Vector_(3, 1.3, 1.4, 1.5));
	delta.insert("l1", Vector_(2, 1.0, 1.1));
	delta.insert("l2", Vector_(2, 1.3, 1.4));

	TupPairConfig expected;
	expected.insert(x1k, expmap(xA1, Vector_(3, 1.0, 1.1, 1.2)));
	expected.insert(x2k, expmap(xA2, Vector_(3, 1.3, 1.4, 1.5)));
	expected.insert(l1k, Point2(5.0, 6.1));
	expected.insert(l2k, Point2(10.3, 11.4));

	CHECK(assert_equal(expected, expmap(config1, delta)));
	CHECK(assert_equal(delta, logmap(config1, expected)));
}

/* ************************************************************************* */
TEST( testFusionTupleConfig, configN)
{
	typedef FusionTupleConfig2<PoseConfig, PointConfig> ConfigA;
	PointConfig expPointConfig;
	expPointConfig.insert(l1, l1_val);
	expPointConfig.insert(l2, l2_val);

	PoseConfig expPoseConfig;
	expPoseConfig.insert(x1, x1_val);
	expPoseConfig.insert(x2, x2_val);

	ConfigA cfg1;
	EXPECT(cfg1.empty());

	ConfigA cfg2(expPoseConfig, expPointConfig);

	EXPECT(assert_equal(expPoseConfig, cfg2.config<PoseConfig>()));
	EXPECT(assert_equal(expPointConfig, cfg2.config<PointConfig>()));

	EXPECT(assert_equal(expPoseConfig, cfg2.first()));
	EXPECT(assert_equal(expPointConfig, cfg2.second()));

	ConfigA cfg3(cfg2);
	EXPECT(assert_equal(cfg2, cfg3));
}

/* ************************************************************************* */
TEST( testFusionTupleConfig, basic_factor)
{
	// planar example system
	typedef FusionTupleConfig2<PoseConfig, PointConfig> Config; // pair config
	typedef FusionTupleConfig1<PoseConfig> TestPoseConfig;
	typedef NonlinearFactorGraph<Config> Graph;
	typedef NonlinearOptimizer<Graph,Config> Optimizer;

	// Factors
//	typedef PriorFactor<TestPoseConfig, PoseKey, Pose2> Prior; // fails to add to graph
	typedef PriorFactor<Config, PoseKey> Prior;
	typedef BetweenFactor<Config, PoseKey> Odometry;
	typedef BearingRangeFactor<Config, PoseKey, PointKey> BearingRange;

	PoseKey pose1k(1), pose2k(2), pose3k(3);
	Pose2 pose1, pose2(2.0, 0.0, 0.0), pose3(4.0, 0.0, 0.0);
	SharedDiagonal prior_model = noiseModel::Isotropic::Sigma(3, 0.1);
	SharedDiagonal odom_model = noiseModel::Diagonal::Sigmas(Vector_(3, 0.2, 0.2, 0.1));

	Graph graph;
	graph.add(Prior(pose1k, pose1, prior_model));
	graph.add(Odometry(pose1k, pose2k, between(pose1, pose2), odom_model));
	graph.add(Odometry(pose2k, pose3k, between(pose2, pose3), odom_model));

	Config init;
	init.insert(pose1k, Pose2(0.2, 0.4, 0.0));
	init.insert(pose2k, Pose2(1.8,-0.4, 0.3));
	init.insert(pose3k, Pose2(4.1, 0.0,-0.2));

	Optimizer::shared_config actual = Optimizer::optimizeLM(graph, init);

	Config expected;
	expected.insert(pose1k, pose1);
	expected.insert(pose2k, pose2);
	expected.insert(pose3k, pose3);

	EXPECT(assert_equal(expected, *actual, tol));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
