/**
 * @file testAsVectorSpace.cpp
 * @brief Test explicit vector-space embeddings of manifold classes.
 * @author Brett Downing
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/geometry/AsVectorSpace.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3Fisheye.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3f.h>

using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(AsVectorSpace<Cal3f>)
GTSAM_CONCEPT_TESTABLE_INST(AsVectorSpace<Cal3_S2>)
GTSAM_CONCEPT_TESTABLE_INST(AsVectorSpace<Cal3DS2>)
GTSAM_CONCEPT_TESTABLE_INST(AsVectorSpace<Cal3Fisheye>)

/* ************************************************************************* */
namespace as_vector_space_construction {

// Verifies supported calibration manifolds construct at the chosen identity.
TEST(AsVectorSpace, Constructors) {
  const AsVectorSpace<Cal3f> cal3f;
  const AsVectorSpace<Cal3_S2> cal3S2;
  const AsVectorSpace<Cal3DS2> cal3DS2;
  const AsVectorSpace<Cal3Fisheye> fisheye;
  EXPECT(cal3f.equals(Cal3f()));
  EXPECT(cal3S2.equals(Cal3_S2()));
  EXPECT(cal3DS2.equals(Cal3DS2()));
  EXPECT(fisheye.equals(Cal3Fisheye()));
}

// Verifies a wrapped value retains its underlying calibration parameters.
TEST(AsVectorSpace, WrapAndViewBase) {
  const Cal3DS2 calibration(9, 8, 7, 6, 5, 4, 3, 2, 1);
  const AsVectorSpace<Cal3DS2> wrapped(calibration);
  const Cal3DS2& base = wrapped;
  EXPECT(base.equals(calibration));
}

// Verifies each adapter satisfies vector-space, group, and manifold concepts.
TEST(AsVectorSpace, Concepts) {
  GTSAM_CONCEPT_ASSERT(IsVectorSpace<AsVectorSpace<Cal3f>>);
  GTSAM_CONCEPT_ASSERT(IsGroup<AsVectorSpace<Cal3f>>);
  GTSAM_CONCEPT_ASSERT(IsManifold<AsVectorSpace<Cal3f>>);
  GTSAM_CONCEPT_ASSERT(IsVectorSpace<AsVectorSpace<Cal3_S2>>);
  GTSAM_CONCEPT_ASSERT(IsVectorSpace<AsVectorSpace<Cal3DS2>>);
  GTSAM_CONCEPT_ASSERT(IsVectorSpace<AsVectorSpace<Cal3Fisheye>>);
}

}  // namespace as_vector_space_construction
/* ************************************************************************* */

/* ************************************************************************* */
namespace as_vector_space_invariants {

template <class Calibration>
bool invariantsHold(const typename traits<Calibration>::TangentVector& first,
                    const typename traits<Calibration>::TangentVector& second) {
  const AsVectorSpace<Calibration> a(first);
  const AsVectorSpace<Calibration> b(second);
  return check_group_invariants(a, b) && check_manifold_invariants(a, b);
}

// Verifies the one-parameter calibration embedding obeys all invariants.
TEST(AsVectorSpace, Cal3fInvariants) {
  traits<Cal3f>::TangentVector first;
  traits<Cal3f>::TangentVector second;
  first << 3.0;
  second << 5.0;
  EXPECT(invariantsHold<Cal3f>(first, second));
}

// Verifies the five-parameter calibration embedding obeys all invariants.
TEST(AsVectorSpace, Cal3S2Invariants) {
  traits<Cal3_S2>::TangentVector first;
  traits<Cal3_S2>::TangentVector second;
  first << 3, 4, 5, 6, 7;
  second << 5, 6, 7, 8, 9;
  EXPECT(invariantsHold<Cal3_S2>(first, second));
}

// Verifies the distortion calibration embedding obeys all invariants.
TEST(AsVectorSpace, Cal3DS2Invariants) {
  traits<Cal3DS2>::TangentVector first;
  traits<Cal3DS2>::TangentVector second;
  first << 3, 4, 5, 6, 7, 8, 9, 10, 11;
  second << 5, 6, 7, 8, 9, 10, 11, 12, 13;
  EXPECT(invariantsHold<Cal3DS2>(first, second));
}

// Verifies the fisheye calibration embedding obeys all invariants.
TEST(AsVectorSpace, Cal3FisheyeInvariants) {
  traits<Cal3Fisheye>::TangentVector first;
  traits<Cal3Fisheye>::TangentVector second;
  first << 3, 4, 5, 6, 7, 8, 9, 10, 11;
  second << 5, 6, 7, 8, 9, 10, 11, 12, 13;
  EXPECT(invariantsHold<Cal3Fisheye>(first, second));
}

}  // namespace as_vector_space_invariants
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
