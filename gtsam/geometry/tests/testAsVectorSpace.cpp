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
#include <gtsam/geometry/Cal3.h>
#include <gtsam/geometry/Cal3Fisheye.h>
#include <gtsam/geometry/AsVectorSpace.h>


using namespace gtsam;


//GTSAM_CONCEPT_TESTABLE_INST(AsVectorSpace<Cal3>)
GTSAM_CONCEPT_TESTABLE_INST(AsVectorSpace<Cal3Fisheye>)
//GTSAM_CONCEPT_LIE_INST(StampedPoint3)

//******************************************************************************
//TEST( AsVectorSpace , Cal3Constructor) {
//  AsVectorSpace<Cal3> p;
//}
//******************************************************************************
TEST( AsVectorSpace , Cal3FisheyeConstructor) {
  AsVectorSpace<Cal3Fisheye> p;
}

//******************************************************************************
//TEST( AsVectorSpace , Cal3UpCast) {
//  Cal3 q();
//  AsVectorSpace<Cal3> p = q;
//}
//******************************************************************************
TEST( AsVectorSpace , Cal3FisheyeUpCast) {
  Cal3Fisheye q = Cal3Fisheye();
  AsVectorSpace<Cal3Fisheye> p(q);// = q;
}

//******************************************************************************
//TEST( AsVectorSpace , Cal3DownCast) {
//  AsVectorSpace<Cal3> p();
//  const Cal3& q = p;
//}
//******************************************************************************
TEST( AsVectorSpace , Cal3FisheyeDownCast) {
  AsVectorSpace<Cal3Fisheye> p = AsVectorSpace<Cal3Fisheye>();
  const Cal3Fisheye& q = p;
}


//******************************************************************************
//TEST( AsVectorSpace , Cal3Concept) {
//  GTSAM_CONCEPT_ASSERT(IsManifold<AsVectorSpace<Cal3>>);
//  GTSAM_CONCEPT_ASSERT(IsGroup<AsVectorSpace<Cal3>>);
//  GTSAM_CONCEPT_ASSERT(IsVectorSpace<AsVectorSpace<Cal3>>);
//}

//******************************************************************************
TEST( AsVectorSpace , Cal3FisheyeConcept) {
  GTSAM_CONCEPT_ASSERT(IsManifold<AsVectorSpace<Cal3Fisheye>>);
  GTSAM_CONCEPT_ASSERT(IsGroup<AsVectorSpace<Cal3Fisheye>>);
  GTSAM_CONCEPT_ASSERT(IsVectorSpace<AsVectorSpace<Cal3Fisheye>>);
}


//******************************************************************************
//TEST( AsVectorSpace , Cal3Invariants) {
//  traits<Cal3>::TangentVector va, vb;
//  va << 3,4,5,6,7;
//  vb << 5,6,7,8,9;
//  AsVectorSpace<Cal3> ca(va), cb(vb);
//  EXPECT(check_group_invariants(ca, cb));
//  EXPECT(check_manifold_invariants(ca, cb));
//}


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
