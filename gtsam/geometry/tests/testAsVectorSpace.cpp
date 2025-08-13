/**
 * @file    testCartesinProduct.cpp
 * @brief   Test vector space properties of extrapolated manifolds
 * @author  Brett Downing
 * @date    April 2025
 */



#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/geometry/Cal3f.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3Fisheye.h>
#include <gtsam/geometry/AsVectorSpace.h>


using namespace gtsam;


GTSAM_CONCEPT_TESTABLE_INST(AsVectorSpace<Cal3f>)
GTSAM_CONCEPT_TESTABLE_INST(AsVectorSpace<Cal3_S2>)
GTSAM_CONCEPT_TESTABLE_INST(AsVectorSpace<Cal3DS2>)
GTSAM_CONCEPT_TESTABLE_INST(AsVectorSpace<Cal3Fisheye>)

//******************************************************************************
TEST( AsVectorSpace , Cal3fConstructor) {
  AsVectorSpace<Cal3f> p;
}
//******************************************************************************
TEST( AsVectorSpace , Cal3DS2Constructor) {
  AsVectorSpace<Cal3f> p;
}
//******************************************************************************
TEST( AsVectorSpace , Cal3_S2Constructor) {
  AsVectorSpace<Cal3_S2> p;
}
//******************************************************************************
TEST( AsVectorSpace , Cal3FisheyeConstructor) {
  AsVectorSpace<Cal3Fisheye> p;
}

//******************************************************************************
TEST( AsVectorSpace , Cal3fUpCast) {
  Cal3f q = Cal3f();
  AsVectorSpace<Cal3f> p(q);
}
//******************************************************************************
TEST( AsVectorSpace , Cal3_S2UpCast) {
  Cal3_S2 q = Cal3_S2();
  AsVectorSpace<Cal3_S2> p(q);
}
//******************************************************************************
TEST( AsVectorSpace , Cal3DS2UpCast) {
  Cal3DS2 q = Cal3DS2();
  AsVectorSpace<Cal3DS2> p(q);
}
//******************************************************************************
TEST( AsVectorSpace , Cal3FisheyeUpCast) {
  Cal3Fisheye q = Cal3Fisheye();
  AsVectorSpace<Cal3Fisheye> p(q);
}

//******************************************************************************
TEST( AsVectorSpace , Cal3fDownCast) {
  AsVectorSpace<Cal3f> p = AsVectorSpace<Cal3f>();
  const Cal3f& q = p;
}
//******************************************************************************
TEST( AsVectorSpace , Cal3_S2DownCast) {
  AsVectorSpace<Cal3_S2> p = AsVectorSpace<Cal3_S2>();
  const Cal3_S2& q = p;
}
//******************************************************************************
TEST( AsVectorSpace , Cal3DS2DownCast) {
  AsVectorSpace<Cal3DS2> p = AsVectorSpace<Cal3DS2>();
  const Cal3DS2& q = p;
}
//******************************************************************************
TEST( AsVectorSpace , Cal3FisheyeDownCast) {
  AsVectorSpace<Cal3Fisheye> p = AsVectorSpace<Cal3Fisheye>();
  const Cal3Fisheye& q = p;
}


//******************************************************************************
TEST( AsVectorSpace , Cal3fConcept) {
  GTSAM_CONCEPT_ASSERT(IsManifold<AsVectorSpace<Cal3f>>);
  GTSAM_CONCEPT_ASSERT(IsGroup<AsVectorSpace<Cal3f>>);
  GTSAM_CONCEPT_ASSERT(IsVectorSpace<AsVectorSpace<Cal3f>>);
}
//******************************************************************************
TEST( AsVectorSpace , Cal3_S2Concept) {
  GTSAM_CONCEPT_ASSERT(IsManifold<AsVectorSpace<Cal3_S2>>);
  GTSAM_CONCEPT_ASSERT(IsGroup<AsVectorSpace<Cal3_S2>>);
  GTSAM_CONCEPT_ASSERT(IsVectorSpace<AsVectorSpace<Cal3_S2>>);
}
//******************************************************************************
TEST( AsVectorSpace , Cal3DS2Concept) {
  GTSAM_CONCEPT_ASSERT(IsManifold<AsVectorSpace<Cal3DS2>>);
  GTSAM_CONCEPT_ASSERT(IsGroup<AsVectorSpace<Cal3DS2>>);
  GTSAM_CONCEPT_ASSERT(IsVectorSpace<AsVectorSpace<Cal3DS2>>);
}
//******************************************************************************
TEST( AsVectorSpace , Cal3FisheyeConcept) {
  GTSAM_CONCEPT_ASSERT(IsManifold<AsVectorSpace<Cal3Fisheye>>);
  GTSAM_CONCEPT_ASSERT(IsGroup<AsVectorSpace<Cal3Fisheye>>);
  GTSAM_CONCEPT_ASSERT(IsVectorSpace<AsVectorSpace<Cal3Fisheye>>);
}


//******************************************************************************
TEST( AsVectorSpace , Cal3fInvariants) {
  traits<Cal3f>::TangentVector va, vb;
  va << 3;
  vb << 5;
  AsVectorSpace<Cal3f> ca(va), cb(vb);
  EXPECT(check_group_invariants(ca, cb));
  EXPECT(check_manifold_invariants(ca, cb));
}
//******************************************************************************
TEST( AsVectorSpace , Cal3_S2Invariants) {
  traits<Cal3_S2>::TangentVector va, vb;
  va << 3,4,5,6,7;
  vb << 5,6,7,8,9;
  AsVectorSpace<Cal3_S2> ca(va), cb(vb);
  EXPECT(check_group_invariants(ca, cb));
  EXPECT(check_manifold_invariants(ca, cb));
}
//******************************************************************************
TEST( AsVectorSpace , Cal3DS2Invariants) {
  traits<Cal3DS2>::TangentVector va, vb;
  va << 3,4,5,6,7,8,9,10,11;
  vb << 5,6,7,8,9,10,11,12,13;
  AsVectorSpace<Cal3DS2> ca(va), cb(vb);
  EXPECT(check_group_invariants(ca, cb));
  EXPECT(check_manifold_invariants(ca, cb));
}
//******************************************************************************
TEST( AsVectorSpace , Cal3FisheyeInvariants) {
  traits<Cal3Fisheye>::TangentVector va, vb;
  va << 3,4,5,6,7,8,9,10,11;
  vb << 5,6,7,8,9,10,11,12,13;
  AsVectorSpace<Cal3Fisheye> ca(va), cb(vb);
  EXPECT(check_group_invariants(ca, cb));
  EXPECT(check_manifold_invariants(ca, cb));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
