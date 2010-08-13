/**
 * @file testFusionTupleConfig.cpp
 * @author Alex Cunningham
 */

#include <iostream>

#include <CppUnitLite/TestHarness.h>

#include <Pose2.h>
#include <Point2.h>
#include <Pose3.h>
#include <Point3.h>
#include <Key.h>
#include <LieConfig-inl.h>

#include <FusionTupleConfig.h>

using namespace boost;
using namespace gtsam;
using namespace std;

static const double tol = 1e-5;

typedef TypedSymbol<Pose2, 'x'> PoseKey;
typedef TypedSymbol<Point2, 'l'> PointKey;
typedef LieConfig<PoseKey, Pose2> PoseConfig;
typedef LieConfig<PointKey, Point2> PointConfig;

// some generic poses, points and keys
PoseKey x1(1), x2(2);
Pose2 x1_val(1.0, 2.0, 0.3), x2_val(2.0, 3.0, 0.4);
PointKey l1(1), l2(2);
Point2 l1_val(1.0, 2.0), l2_val(3.0, 4.0);

typedef FusionTupleConfig<fusion::set<PointConfig> > TupPointConfig;
typedef FusionTupleConfig<fusion::set<PoseConfig> > TupPoseConfig;
typedef FusionTupleConfig<fusion::set<PoseConfig, PointConfig> > TupPairConfig;

/* ************************************************************************* */
TEST( testFusionTupleConfig, basic_config ) {

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
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
