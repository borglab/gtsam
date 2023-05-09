/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testLieScalar.cpp
 * @author Kai Ni
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/deprecated/LieScalar.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Manifold.h>

using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(LieScalar)
GTSAM_CONCEPT_LIE_INST(LieScalar)

const double tol=1e-9;

//******************************************************************************
TEST(LieScalar , Concept) {
  BOOST_CONCEPT_ASSERT((IsGroup<LieScalar>));
  BOOST_CONCEPT_ASSERT((IsManifold<LieScalar>));
  BOOST_CONCEPT_ASSERT((IsLieGroup<LieScalar>));
}

//******************************************************************************
TEST(LieScalar , Invariants) {
  LieScalar lie1(2), lie2(3);
  CHECK(check_group_invariants(lie1, lie2));
  CHECK(check_manifold_invariants(lie1, lie2));
}

/* ************************************************************************* */
TEST( testLieScalar, construction ) {
  double d = 2.;
  LieScalar lie1(d), lie2(d);

  EXPECT_DOUBLES_EQUAL(2., lie1.value(),tol);
  EXPECT_DOUBLES_EQUAL(2., lie2.value(),tol);
  EXPECT(traits<LieScalar>::dimension == 1);
  EXPECT(assert_equal(lie1, lie2));
}

/* ************************************************************************* */
TEST( testLieScalar, localCoordinates ) {
  LieScalar lie1(1.), lie2(3.);

  Vector1 actual = traits<LieScalar>::Local(lie1, lie2);
  EXPECT( assert_equal((Vector)(Vector(1) << 2).finished(), actual));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
