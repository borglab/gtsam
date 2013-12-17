/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file testSphere2.cpp
 * @date Feb 03, 2012
 * @author Can Erdogan
 * @brief Tests the Sphere2 class
 */

#include <gtsam/base/Testable.h>
#include <gtsam/geometry/Sphere2.h>
#include <CppUnitLite/TestHarness.h>

using namespace gtsam;
using namespace std;

GTSAM_CONCEPT_TESTABLE_INST(Sphere2)
GTSAM_CONCEPT_MANIFOLD_INST(Sphere2)

/// Returns a random vector
inline static Vector randomVector(const Vector& minLimits,
    const Vector& maxLimits) {

  // Get the number of dimensions and create the return vector
  size_t numDims = dim(minLimits);
  Vector vector = zero(numDims);

  // Create the random vector
  for (size_t i = 0; i < numDims; i++) {
    double range = maxLimits(i) - minLimits(i);
    vector(i) = (((double) rand()) / RAND_MAX) * range + minLimits(i);
  }
  return vector;
}

/* ************************************************************************* */
// Let x and y be two Sphere2's.
// The equality x.localCoordinates(x.retract(v)) == v should hold.
TEST(Sphere2, localCoordinates_retract) {

  size_t numIterations = 10000;
  Vector minSphereLimit = Vector_(3, -1.0, -1.0, -1.0), maxSphereLimit =
      Vector_(3, 1.0, 1.0, 1.0);
  Vector minXiLimit = Vector_(2, -1.0, -1.0), maxXiLimit = Vector_(2, 1.0, 1.0);
  for (size_t i = 0; i < numIterations; i++) {

    // Sleep for the random number generator (TODO?: Better create all of them first).
    sleep(0);

    // Create the two Sphere2s.
    // NOTE: You can not create two totally random Sphere2's because you cannot always compute
    // between two any Sphere2's. (For instance, they might be at the different sides of the circle).
    Sphere2 s1(Point3(randomVector(minSphereLimit, maxSphereLimit)));
//		Sphere2 s2 (Point3(randomVector(minSphereLimit, maxSphereLimit)));
    Vector v12 = randomVector(minXiLimit, maxXiLimit);
    Sphere2 s2 = s1.retract(v12);

    // Check if the local coordinates and retract return the same results.
    Vector actual_v12 = s1.localCoordinates(s2);
    EXPECT(assert_equal(v12, actual_v12, 1e-3));
    Sphere2 actual_s2 = s1.retract(actual_v12);
    EXPECT(assert_equal(s2, actual_s2, 1e-3));
  }
}

/* ************************************************************************* */
int main() {
  srand(time(NULL));
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
