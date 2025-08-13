/**
 * @file    testCartesinProduct.cpp
 * @brief   Test manifold properties of combined manifolds
 * @author  Brett Downing
 * @date    April 2025
 */



#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/CartesianProduct.h>

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/AsVectorSpace.h>


using namespace gtsam;

// These types are mainly testing examples,
// see Event and Gal3 for more capable manifolds
typedef CartesianProduct<Point3, double> StampedPoint3;
typedef CartesianProduct<Pose3, double> StampedPose3;
// split variant of pose3 allows comparing interpolate to interpolateRT
typedef CartesianProduct<Rot3,Point3> SplitPose3;
typedef CartesianProduct<SplitPose3, double> StampedSplitPose3;
// bind a multiplicative group to an additive group
typedef CartesianProduct<Pose3, AsVectorSpace<Cal3DS2>> CameraPose;



GTSAM_CONCEPT_TESTABLE_INST(StampedPoint3)
GTSAM_CONCEPT_LIE_INST(StampedPoint3)

GTSAM_CONCEPT_TESTABLE_INST(CameraPose)
GTSAM_CONCEPT_LIE_INST(CameraPose)



static Point3 P(0.2, 0.7, -2);
static StampedPoint3 S(P, 0.1);
//******************************************************************************
TEST( CartesianProduct , StampedPoint3Constructor) {
  StampedPoint3 p;
}
//******************************************************************************
TEST( CartesianProduct , CameraPoseConstructor) {
  CameraPose p;
}

//******************************************************************************
TEST( CartesianProduct , StampedPoint3Concept) {
  GTSAM_CONCEPT_ASSERT(IsGroup<StampedPoint3>);
  GTSAM_CONCEPT_ASSERT(IsManifold<StampedPoint3>);
  //GTSAM_CONCEPT_ASSERT(IsVectorSpace<StampedPoint3>);
}
//******************************************************************************
TEST( CartesianProduct , CameraPoseConcept) {
  GTSAM_CONCEPT_ASSERT(IsGroup<CameraPose>);
  GTSAM_CONCEPT_ASSERT(IsManifold<CameraPose>);
  //GTSAM_CONCEPT_ASSERT(IsVectorSpace<CameraPose>);
}


//******************************************************************************
TEST( CartesianProduct , StampedPoint3Invariants) {
  StampedPoint3 p1(Point3(1, 2, 3),7), p2(Point3(4, 5, 6),8);
  EXPECT(check_group_invariants(p1, p2));
  EXPECT(check_manifold_invariants(p1, p2));
}
//******************************************************************************
TEST( CartesianProduct , CameraPoseInvariants) {
  CameraPose p1(Pose3(Rot3::Rodrigues(0.3,0.2,0.1), Point3(1, 2, 3)), AsVectorSpace<Cal3DS2>(Cal3DS2(9,8,7,6,5,4,3,2,1)) );
  CameraPose p2(Pose3(Rot3::Rodrigues(0.1,0.2,0.3), Point3(4, 5, 6)), AsVectorSpace<Cal3DS2>(Cal3DS2(6,7,8,9,1,2,3,4,5)) );
  EXPECT(check_group_invariants(p1, p2));
  EXPECT(check_manifold_invariants(p1, p2));
}
//******************************************************************************

TEST( CartesianProduct, StampedPoint3interpolate) {
  EXPECT(StampedPoint3(Point3(2,2,2), 2).equals(interpolate(
    StampedPoint3(Point3(1,2,3),0),
    StampedPoint3(Point3(3,2,1),4), 0.5), 1e-9));
}



//******************************************************************************
TEST( CartesianProduct , StampedPose3Concept) {
  GTSAM_CONCEPT_ASSERT(IsGroup<StampedPose3>);
  GTSAM_CONCEPT_ASSERT(IsManifold<StampedPose3>);
  //GTSAM_CONCEPT_ASSERT(IsVectorSpace<StampedPose3>);
}

//******************************************************************************
TEST( CartesianProduct , StampedPose3Invariants) {
  StampedPose3  p1(Pose3(Rot3::Rodrigues(0.3,0.2,0.1), Point3(1, 2, 3)),7),
                p2(Pose3(Rot3::Rodrigues(0.1,0.2,0.3), Point3(4, 5, 6)),8);
  EXPECT(check_group_invariants(p1, p2));
  EXPECT(check_manifold_invariants(p1, p2));
}
//******************************************************************************

TEST( CartesianProduct, StampedPose3interpolate) {
  EXPECT(StampedPose3(Pose3(Rot3::Rx(0), Point3(2,2,2)), 2).equals(interpolate(
    StampedPose3(Pose3(Rot3::Rx(0), Point3(1,2,3)),0),
    StampedPose3(Pose3(Rot3::Rx(0), Point3(3,2,1)),4), 0.5), 1e-7));
  EXPECT(StampedPose3(Pose3(Rot3::Rx(0.2), Point3(0,0,0)), 2).equals(interpolate(
    StampedPose3(Pose3(Rot3::Rx(0), Point3(0,0,0)),0),
    StampedPose3(Pose3(Rot3::Rx(0.4), Point3(0,0,0)),4), 0.5), 1e-9));
}



//******************************************************************************
TEST( CartesianProduct , StampedSplitPose3Concept) {
  GTSAM_CONCEPT_ASSERT(IsGroup<StampedSplitPose3>);
  GTSAM_CONCEPT_ASSERT(IsManifold<StampedSplitPose3>);
  //GTSAM_CONCEPT_ASSERT(IsVectorSpace<StampedSplitPose3>);
}

//******************************************************************************
TEST( CartesianProduct , StampedSplitPose3Invariants) {
  StampedSplitPose3 p1(SplitPose3(Rot3::Rodrigues(0.3,0.2,0.1), Point3(1, 2, 3)),7),
                    p2(SplitPose3(Rot3::Rodrigues(0.1,0.2,0.3), Point3(4, 5, 6)),8);
  EXPECT(check_group_invariants(p1, p2));
  EXPECT(check_manifold_invariants(p1, p2));
}
//******************************************************************************

TEST( CartesianProduct, StampedSplitPose3interpolate) {
  EXPECT(StampedSplitPose3(SplitPose3(Rot3::Rx(0.2), Point3(2,2,2)), 2).equals(interpolate(
    StampedSplitPose3(SplitPose3(Rot3::Rx(0), Point3(1,2,3)),0),
    StampedSplitPose3(SplitPose3(Rot3::Rx(0.4), Point3(3,2,1)),4), 0.5), 1e-9));
}


/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

