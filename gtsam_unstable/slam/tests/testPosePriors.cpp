/**
 * @file testPosePriors.cpp
 *
 * @brief Tests partial priors on poses
 *
 * @date Jun 14, 2012
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam_unstable/slam/PoseRotationPrior.h>
#include <gtsam_unstable/slam/PoseTranslationPrior.h>

#include <gtsam/slam/planarSLAM.h>
#include <gtsam/slam/visualSLAM.h>

#include <gtsam/nonlinear/Symbol.h>

using namespace gtsam;

Key x1 = symbol_shorthand::X(1);

const SharedNoiseModel model1 = noiseModel::Diagonal::Sigmas(Vector_(1, 0.1));
const SharedNoiseModel model2 = noiseModel::Diagonal::Sigmas(Vector_(2, 0.1, 0.2));
const SharedNoiseModel model3 = noiseModel::Diagonal::Sigmas(Vector_(3, 0.1, 0.2, 0.3));

/* ************************************************************************* */
TEST( testPoseTranslationPrior, planar) {

	typedef PoseTranslationPrior<Pose2> PlanarTPrior;
	PlanarTPrior actTprior(x1, Point2(2.0, 3.0), model2);
	EXPECT(assert_equal(Point2(2.0, 3.0), actTprior.priorTranslation()));
	Matrix expH(2,3); expH << eye(2,2), zeros(2,1);
	EXPECT(assert_equal(eye(2,3), actTprior.H()));
}

/* ************************************************************************* */
TEST( testPoseTranslationPrior, visual) {

	typedef PoseTranslationPrior<Pose3> VisualTPrior;
	VisualTPrior actPose3Tprior(x1, Point3(2.0, 3.0, 4.0), model3);
	EXPECT(assert_equal(Point3(2.0, 3.0, 4.0), actPose3Tprior.priorTranslation()));
	Matrix expH(3,6); expH << zeros(3,3), eye(3,3);
	EXPECT(assert_equal(expH, actPose3Tprior.H()));

//	typedef PoseTranslationPrior<SimpleCamera> CamTPrior;
//	CamTPrior actCamTprior(x1, Point3(2.0, 3.0, 4.0), model3);
}

/* ************************************************************************* */
TEST( testPoseRotationPrior, planar) {

	typedef PoseRotationPrior<Pose2> PlanarRPrior;
	PlanarRPrior actRprior(x1, Rot2::fromDegrees(30.0), model1);
	EXPECT(assert_equal(Rot2::fromDegrees(30.0), actRprior.priorRotation()));
	Matrix expH(1,3); expH << 0.0, 0.0, 1.0;
	EXPECT(assert_equal(expH, actRprior.H()));
}

/* ************************************************************************* */
TEST( testPoseRotationPrior, visual) {

	typedef PoseRotationPrior<Pose3> VisualRPrior;
	VisualRPrior actPose3Rprior(x1, Rot3::RzRyRx(0.1, 0.2, 0.3), model3);
	EXPECT(assert_equal(Rot3::RzRyRx(0.1, 0.2, 0.3), actPose3Rprior.priorRotation()));
	Matrix expH(3,6); expH << eye(3,3), zeros(3,3);
	EXPECT(assert_equal(expH, actPose3Rprior.H()));

//	typedef testPoseRotationPrior<SimpleCamera> CamRPrior;
//	CamRPrior actCamRprior(x1, Rot3::RzRyRx(0.1, 0.2, 0.3), model3);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
