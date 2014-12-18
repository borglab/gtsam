/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------1------------------------------------------- */

/**
 * @file testExpression.cpp
 * @date September 18, 2014
 * @author Frank Dellaert
 * @author Paul Furgale
 * @brief unit tests for Block Automatic Differentiation
 */

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/base/Manifold.h>
#include <gtsam/base/Testable.h>

#undef CHECK
#include <CppUnitLite/TestHarness.h>

#include <boost/assign/list_of.hpp>
using boost::assign::list_of;
using boost::assign::map_list_of;

using namespace std;
using namespace gtsam;

// The DefaultChart of Camera below is laid out like Snavely's 9-dim vector
typedef PinholeCamera<Cal3Bundler> Camera;

/* ************************************************************************* */
// is_manifold
TEST(Manifold, IsManifold) {

  //BOOST_CONCEPT_ASSERT((IsManifold<int>)); // integer is not a manifold
  BOOST_CONCEPT_ASSERT((IsManifold<Point2>));
  BOOST_CONCEPT_ASSERT((IsManifold<Matrix24>));
  BOOST_CONCEPT_ASSERT((IsManifold<double>));
  BOOST_CONCEPT_ASSERT((IsManifold<Camera>));

  // dynamic not working yet
  //BOOST_CONCEPT_ASSERT((IsManifold<Vector>));
  //BOOST_CONCEPT_ASSERT((IsManifold<Matrix>));

}

/* ************************************************************************* */
// dimension
TEST(Manifold, _dimension) {
  //using namespace traits;
  EXPECT_LONGS_EQUAL(2, traits_x<Point2>::dimension);
  EXPECT_LONGS_EQUAL(8, traits_x<Matrix24>::dimension);
  EXPECT_LONGS_EQUAL(1, traits_x<double>::dimension);
//  EXPECT_LONGS_EQUAL(Eigen::Dynamic, traits_x<Vector>::dimension);
//  EXPECT_LONGS_EQUAL(Eigen::Dynamic, traits_x<Matrix>::dimension);
}

/* ************************************************************************* */
// charts
TEST(Manifold, DefaultChart) {

  //DefaultChart<Point2> chart1;
  EXPECT(traits_x<Point2>::Local(Point2(0, 0), Point2(1, 0)) == Vector2(1, 0));
  EXPECT(traits_x<Point2>::Retract(Point2(0, 0), Vector2(1, 0)) == Point2(1, 0));

  Vector v2(2);
  v2 << 1, 0;
  //DefaultChart<Vector2> chart2;
  EXPECT(assert_equal(v2, traits_x<Vector2>::Local(Vector2(0, 0), Vector2(1, 0))));
  EXPECT(traits_x<Vector2>::Retract(Vector2(0, 0), v2) == Vector2(1, 0));

  {
    typedef Matrix2 ManifoldPoint;
    ManifoldPoint m;
    //DefaultChart<ManifoldPoint> chart;
    m << 1, 3,
         2, 4;
    // m as per default is in column-major storage mode. So this yields a linear representation of (1, 2, 3, 4)!
    EXPECT(assert_equal(Vector(Vector4(1, 2, 3, 4)), Vector(traits_x<ManifoldPoint>::Local(ManifoldPoint::Zero(), m))));
    EXPECT(traits_x<ManifoldPoint>::Retract(m, Vector4(1, 2, 3, 4)) == 2 * m);
  }

  {
    typedef Eigen::Matrix<double, 1, 2> ManifoldPoint;
    ManifoldPoint m;
    //DefaultChart<ManifoldPoint> chart;
    m << 1, 2;
    EXPECT(assert_equal(Vector(Vector2(1, 2)), Vector(traits_x<ManifoldPoint>::Local(ManifoldPoint::Zero(), m))));
    EXPECT(traits_x<ManifoldPoint>::Retract(m, Vector2(1, 2)) == 2 * m);
  }

  {
    typedef Eigen::Matrix<double, 1, 1> ManifoldPoint;
    ManifoldPoint m;
    //DefaultChart<ManifoldPoint> chart;
    m << 1;
    EXPECT(assert_equal(Vector(ManifoldPoint::Ones()), Vector(traits_x<ManifoldPoint>::Local(ManifoldPoint::Zero(), m))));
    EXPECT(traits_x<ManifoldPoint>::Retract(m, ManifoldPoint::Ones()) == 2 * m);
  }

  //DefaultChart<double> chart3;
  Vector v1(1);
  v1 << 1;
  EXPECT(assert_equal(v1, traits_x<double>::Local(0, 1)));
  EXPECT_DOUBLES_EQUAL(traits_x<double>::Retract(0, v1), 1, 1e-9);

  // Dynamic does not work yet !
  Vector z = zero(2), v(2);
  v << 1, 0;
  //DefaultChart<Vector> chart4;
//  EXPECT(assert_equal(traits_x<Vector>::Local(z, v), v));
//  EXPECT(assert_equal(traits_x<Vector>::Retract(z, v), v));

  Vector v3(3);
  v3 << 1, 1, 1;
  Rot3 I = Rot3::identity();
  Rot3 R = I.retract(v3);
  //DefaultChart<Rot3> chart5;
  EXPECT(assert_equal(v3, traits_x<Rot3>::Local(I, R)));
  EXPECT(assert_equal(traits_x<Rot3>::Retract(I, v3), R));
  // Check zero vector
  //DefaultChart<Rot3> chart6;
  EXPECT(assert_equal(zero(3), traits_x<Rot3>::Local(R, R)));
}

/* ************************************************************************* */
// zero
//TEST(Manifold, _zero) {
//  EXPECT(assert_equal(Pose3(), traits::zero<Pose3>::value()));
//  Cal3Bundler cal(0, 0, 0);
//  EXPECT(assert_equal(cal, traits::zero<Cal3Bundler>::value()));
//  EXPECT(assert_equal(Camera(Pose3(), cal), traits::zero<Camera>::value()));
//  EXPECT(assert_equal(Point2(), traits::zero<Point2>::value()));
//  EXPECT(assert_equal(Matrix(Matrix24::Zero()), Matrix(traits::zero<Matrix24>::value())));
//  EXPECT_DOUBLES_EQUAL(0.0, traits::zero<double>::value(), 0.0);
//}
//
/* ************************************************************************* */
// identity
TEST(Manifold, _identity) {
  EXPECT(assert_equal(Pose3(), traits_x<Pose3>::Identity()));
//  BOOST_CONCEPT_ASSERT((IsLieGroup<Cal3Bundler>));
//  EXPECT(assert_equal(Cal3Bundler(), traits_x<Cal3Bundler>::Identity()));
//  BOOST_CONCEPT_ASSERT((IsLieGroup<Camera>));
//  EXPECT(assert_equal(Camera(), traits_x<Camera>::Identity())); // not a lie group
  EXPECT(assert_equal(Point2(), traits_x<Point2>::Identity()));
  EXPECT(assert_equal(Matrix(Matrix24::Zero()), Matrix(traits_x<Matrix24>::Identity())));
  EXPECT_DOUBLES_EQUAL(0.0, traits_x<double>::Identity(), 0.0);
}

/* ************************************************************************* */
// charts
TEST(Manifold, Canonical) {

  Canonical<Point2> chart1;
  EXPECT(chart1.Local(Point2(1, 0))==Vector2(1, 0));
  EXPECT(chart1.Retract(Vector2(1, 0))==Point2(1, 0));

  Vector v2(2);
  v2 << 1, 0;
  Canonical<Vector2> chart2;
  EXPECT(assert_equal(v2, chart2.Local(Vector2(1, 0))));
  EXPECT(chart2.Retract(v2)==Vector2(1, 0));

  Canonical<double> chart3;
  Eigen::Matrix<double, 1, 1> v1;
  v1 << 1;
  EXPECT(chart3.Local(1)==v1);
  EXPECT_DOUBLES_EQUAL(chart3.Retract(v1), 1, 1e-9);

  Canonical<Point3> chart4;
  Point3 point(1, 2, 3);
  Vector v3(3);
  v3 << 1, 2, 3;
  EXPECT(assert_equal(v3, chart4.Local(point)));
  EXPECT(assert_equal(chart4.Retract(v3), point));

  Canonical<Pose3> chart5;
  Pose3 pose(Rot3::identity(), point);
  Vector v6(6);
  v6 << 0, 0, 0, 1, 2, 3;
  EXPECT(assert_equal(v6, chart5.Local(pose)));
  EXPECT(assert_equal(chart5.Retract(v6), pose));

  Canonical<Camera> chart6;
  Cal3Bundler cal0(0, 0, 0);
  Camera camera(Pose3(), cal0);
  Vector z9 = Vector9::Zero();
  EXPECT(assert_equal(z9, chart6.Local(camera)));
  EXPECT(assert_equal(chart6.Retract(z9), camera));

  Cal3Bundler cal; // Note !! Cal3Bundler() != zero<Cal3Bundler>::value()
  Camera camera2(pose, cal);
  Vector v9(9);
  v9 << 0, 0, 0, 1, 2, 3, 1, 0, 0;
  EXPECT(assert_equal(v9, chart6.Local(camera2)));
  EXPECT(assert_equal(chart6.Retract(v9), camera2));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

