/**
 * @file testProductLieGroupGeometry.cpp
 * @brief Test direct products of geometry types.
 * @author Brett Downing
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/ProductLieGroup.h>
#include <gtsam/geometry/AsVectorSpace.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Pose3.h>

using namespace gtsam;

using StampedPoint3 = ProductLieGroup<Point3, double>;
using StampedPose3 = ProductLieGroup<Pose3, double>;
using SplitPose3 = ProductLieGroup<Rot3, Point3>;
using StampedSplitPose3 = ProductLieGroup<SplitPose3, double>;
using CameraPose = ProductLieGroup<Pose3, AsVectorSpace<Cal3DS2>>;

GTSAM_CONCEPT_TESTABLE_INST(StampedPoint3)
GTSAM_CONCEPT_LIE_INST(StampedPoint3)
GTSAM_CONCEPT_TESTABLE_INST(CameraPose)
GTSAM_CONCEPT_LIE_INST(CameraPose)

/* ************************************************************************* */
namespace product_lie_group_geometry_concepts {

// Verifies representative direct products construct at the identity.
TEST(ProductLieGroupGeometry, Constructors) {
  StampedPoint3 stampedPoint;
  CameraPose cameraPose;
  EXPECT(stampedPoint.equals(StampedPoint3::Identity()));
  EXPECT(cameraPose.equals(CameraPose::Identity()));
}

// Verifies representative products satisfy group and manifold concepts.
TEST(ProductLieGroupGeometry, Concepts) {
  GTSAM_CONCEPT_ASSERT(IsGroup<StampedPoint3>);
  GTSAM_CONCEPT_ASSERT(IsManifold<StampedPoint3>);
  GTSAM_CONCEPT_ASSERT(IsGroup<StampedPose3>);
  GTSAM_CONCEPT_ASSERT(IsManifold<StampedPose3>);
  GTSAM_CONCEPT_ASSERT(IsGroup<CameraPose>);
  GTSAM_CONCEPT_ASSERT(IsManifold<CameraPose>);
}

}  // namespace product_lie_group_geometry_concepts
/* ************************************************************************* */

/* ************************************************************************* */
namespace product_lie_group_geometry_invariants {

// Verifies a vector-and-time product obeys group and manifold invariants.
TEST(ProductLieGroupGeometry, StampedPoint3Invariants) {
  const StampedPoint3 first(Point3(1, 2, 3), 7.0);
  const StampedPoint3 second(Point3(4, 5, 6), 8.0);
  EXPECT(check_group_invariants(first, second));
  EXPECT(check_manifold_invariants(first, second));
}

// Verifies a pose-and-calibration product obeys the expected invariants.
TEST(ProductLieGroupGeometry, CameraPoseInvariants) {
  const CameraPose first(
      Pose3(Rot3::Rodrigues(0.3, 0.2, 0.1), Point3(1, 2, 3)),
      AsVectorSpace<Cal3DS2>(Cal3DS2(9, 8, 7, 6, 5, 4, 3, 2, 1)));
  const CameraPose second(
      Pose3(Rot3::Rodrigues(0.1, 0.2, 0.3), Point3(4, 5, 6)),
      AsVectorSpace<Cal3DS2>(Cal3DS2(6, 7, 8, 9, 1, 2, 3, 4, 5)));
  EXPECT(check_group_invariants(first, second));
  EXPECT(check_manifold_invariants(first, second));
}

// Verifies products containing Pose3 obey group and manifold invariants.
TEST(ProductLieGroupGeometry, StampedPose3Invariants) {
  const StampedPose3 first(
      Pose3(Rot3::Rodrigues(0.3, 0.2, 0.1), Point3(1, 2, 3)), 7.0);
  const StampedPose3 second(
      Pose3(Rot3::Rodrigues(0.1, 0.2, 0.3), Point3(4, 5, 6)), 8.0);
  EXPECT(check_group_invariants(first, second));
  EXPECT(check_manifold_invariants(first, second));
}

// Verifies nested direct products preserve the same invariants.
TEST(ProductLieGroupGeometry, NestedInvariants) {
  const StampedSplitPose3 first(
      SplitPose3(Rot3::Rodrigues(0.3, 0.2, 0.1), Point3(1, 2, 3)), 7.0);
  const StampedSplitPose3 second(
      SplitPose3(Rot3::Rodrigues(0.1, 0.2, 0.3), Point3(4, 5, 6)), 8.0);
  EXPECT(check_group_invariants(first, second));
  EXPECT(check_manifold_invariants(first, second));
}

}  // namespace product_lie_group_geometry_invariants
/* ************************************************************************* */

/* ************************************************************************* */
namespace product_lie_group_geometry_interpolation {

// Verifies interpolation acts independently on both product components.
TEST(ProductLieGroupGeometry, Interpolate) {
  const StampedPoint3 first(Point3(1, 2, 3), 0.0);
  const StampedPoint3 second(Point3(3, 2, 1), 4.0);
  const StampedPoint3 expected(Point3(2, 2, 2), 2.0);
  EXPECT(expected.equals(interpolate(first, second, 0.5), 1e-9));

  const StampedPose3 firstPose(Pose3(Rot3::Rx(0.0), Point3(0, 0, 0)), 0.0);
  const StampedPose3 secondPose(Pose3(Rot3::Rx(0.4), Point3(0, 0, 0)), 4.0);
  const StampedPose3 expectedPose(Pose3(Rot3::Rx(0.2), Point3(0, 0, 0)), 2.0);
  EXPECT(expectedPose.equals(interpolate(firstPose, secondPose, 0.5), 1e-9));
}

}  // namespace product_lie_group_geometry_interpolation
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
