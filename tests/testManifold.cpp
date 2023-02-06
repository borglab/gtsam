/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------1------------------------------------------- */

/**
 * @file testManifold.cpp
 * @date September 18, 2014
 * @author Frank Dellaert
 * @author Paul Furgale
 * @brief unit tests for Manifold type machinery
 */

#include <gtsam/base/Manifold.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/base/VectorSpace.h>
#include <gtsam/base/testLie.h>
#include <gtsam/base/Testable.h>

#undef CHECK
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

// The DefaultChart of Camera below is laid out like Snavely's 9-dim vector
typedef PinholeCamera<Cal3Bundler> Camera;

//******************************************************************************
TEST(Manifold, SomeManifoldsGTSAM) {
  //GTSAM_CONCEPT_ASSERT(IsManifold<int>); // integer is not a manifold
  GTSAM_CONCEPT_ASSERT1(IsManifold<Camera>);
  GTSAM_CONCEPT_ASSERT2(IsManifold<Cal3_S2>);
  GTSAM_CONCEPT_ASSERT3(IsManifold<Cal3Bundler>);
  GTSAM_CONCEPT_ASSERT4(IsManifold<Camera>);
}

//******************************************************************************
TEST(Manifold, SomeLieGroupsGTSAM) {
  GTSAM_CONCEPT_ASSERT1(IsLieGroup<Rot2>);
  GTSAM_CONCEPT_ASSERT2(IsLieGroup<Pose2>);
  GTSAM_CONCEPT_ASSERT3(IsLieGroup<Rot3>);
  GTSAM_CONCEPT_ASSERT4(IsLieGroup<Pose3>);
}

//******************************************************************************
TEST(Manifold, SomeVectorSpacesGTSAM) {
  GTSAM_CONCEPT_ASSERT1(IsVectorSpace<double>);
  GTSAM_CONCEPT_ASSERT2(IsVectorSpace<float>);
  GTSAM_CONCEPT_ASSERT3(IsVectorSpace<Point2>);
  GTSAM_CONCEPT_ASSERT4(IsVectorSpace<Matrix24>);
}

//******************************************************************************
// dimension
TEST(Manifold, _dimension) {
  //using namespace traits;
  EXPECT_LONGS_EQUAL(2, traits<Point2>::dimension);
  EXPECT_LONGS_EQUAL(8, traits<Matrix24>::dimension);
  EXPECT_LONGS_EQUAL(1, traits<double>::dimension);
}

//******************************************************************************
TEST(Manifold, Identity) {
  EXPECT_DOUBLES_EQUAL(0.0, traits<double>::Identity(), 0.0);
  EXPECT(assert_equal(Matrix(Matrix24::Zero()), Matrix(traits<Matrix24>::Identity())));
  EXPECT(assert_equal(Pose3(), traits<Pose3>::Identity()));
  EXPECT(assert_equal(Point2(0,0), traits<Point2>::Identity()));
}

//******************************************************************************
// charts
TEST(Manifold, DefaultChart) {

  //DefaultChart<Point2> chart1;
  EXPECT(traits<Point2>::Local(Point2(0, 0), Point2(1, 0)) == Vector2(1, 0));
  EXPECT(traits<Point2>::Retract(Point2(0, 0), Vector2(1, 0)) == Point2(1, 0));

  Vector v2(2);
  v2 << 1, 0;
  //DefaultChart<Vector2> chart2;
  EXPECT(assert_equal(v2, traits<Vector2>::Local(Vector2(0, 0), Vector2(1, 0))));
  EXPECT(traits<Vector2>::Retract(Vector2(0, 0), v2) == Vector2(1, 0));

  {
    typedef Matrix2 ManifoldPoint;
    ManifoldPoint m;
    //DefaultChart<ManifoldPoint> chart;
    m << 1, 3,
         2, 4;
    // m as per default is in column-major storage mode. So this yields a linear representation of (1, 2, 3, 4)!
    EXPECT(assert_equal(Vector(Vector4(1, 2, 3, 4)), Vector(traits<ManifoldPoint>::Local(ManifoldPoint::Zero(), m))));
    EXPECT(traits<ManifoldPoint>::Retract(m, Vector4(1, 2, 3, 4)) == 2 * m);
  }

  {
    typedef Eigen::Matrix<double, 1, 2> ManifoldPoint;
    ManifoldPoint m;
    //DefaultChart<ManifoldPoint> chart;
    m << 1, 2;
    EXPECT(assert_equal(Vector(Vector2(1, 2)), Vector(traits<ManifoldPoint>::Local(ManifoldPoint::Zero(), m))));
    EXPECT(traits<ManifoldPoint>::Retract(m, Vector2(1, 2)) == 2 * m);
  }

  {
    typedef Eigen::Matrix<double, 1, 1> ManifoldPoint;
    ManifoldPoint m;
    //DefaultChart<ManifoldPoint> chart;
    m << 1;
    EXPECT(assert_equal(Vector(ManifoldPoint::Ones()), Vector(traits<ManifoldPoint>::Local(ManifoldPoint::Zero(), m))));
    EXPECT(traits<ManifoldPoint>::Retract(m, ManifoldPoint::Ones()) == 2 * m);
  }

  //DefaultChart<double> chart3;
  Vector v1(1);
  v1 << 1;
  EXPECT(assert_equal(v1, traits<double>::Local(0, 1)));
  EXPECT_DOUBLES_EQUAL(traits<double>::Retract(0, v1), 1, 1e-9);

  // Dynamic does not work yet !
  Vector z = Z_2x1, v(2);
  v << 1, 0;
  //DefaultChart<Vector> chart4;
//  EXPECT(assert_equal(traits<Vector>::Local(z, v), v));
//  EXPECT(assert_equal(traits<Vector>::Retract(z, v), v));

  Vector v3(3);
  v3 << 1, 1, 1;
  Rot3 I = Rot3::Identity();
  Rot3 R = I.retract(v3);
  //DefaultChart<Rot3> chart5;
  EXPECT(assert_equal(v3, traits<Rot3>::Local(I, R)));
  EXPECT(assert_equal(traits<Rot3>::Retract(I, v3), R));
  // Check zero vector
  //DefaultChart<Rot3> chart6;
  EXPECT(assert_equal((Vector) Z_3x1, traits<Rot3>::Local(R, R)));
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************

