/**
 * @file testSimWall2D2D
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>

#include <gtsam_unstable/geometry/SimWall2D.h>

using namespace gtsam;

const double tol = 1e-5;

/* ************************************************************************* */
TEST(testSimWall2D2D, construction ) {
  Point2 a(1.0, 0.0), b(1.0, 2.0);
  SimWall2D wall1(a, b), wall2(a.x(), a.y(), b.x(), b.y());
  EXPECT(assert_equal(a, wall1.a(), tol));
  EXPECT(assert_equal(a, wall2.a(), tol));
  EXPECT(assert_equal(b, wall1.b(), tol));
  EXPECT(assert_equal(b, wall2.b(), tol));
}

/* ************************************************************************* */
TEST(testSimWall2D2D, equals ) {
  Point2 p1(1.0, 0.0), p2(1.0, 2.0), p3(0,0);
  SimWall2D w1(p1, p2), w2(p1, p3);
  EXPECT(assert_equal(w1, w1));
  EXPECT(assert_inequal(w1, w2));
  EXPECT(assert_inequal(w2, w1));
}

/* ************************************************************************* */
TEST(testSimWall2D2D, intersection1 ) {
  SimWall2D w1(2.0, 2.0, 6.0, 2.0), w2(4.0, 4.0, 4.0, 0.0);
  Point2 pt(0,0);
  EXPECT(w1.intersects(w2));
  EXPECT(w2.intersects(w1));
  w1.intersects(w2, pt);
  EXPECT(assert_equal(Point2(4.0, 2.0), pt, tol));
}

/* ************************************************************************* */
TEST(testSimWall2D2D, intersection2 ) {
  SimWall2D traj(7.07107, 7.07107, 0, 0);
  SimWall2D wall(1.5, 3, 1.5, 1);
  EXPECT(wall.intersects(traj));
  EXPECT(traj.intersects(wall));
}

/* ************************************************************************* */
TEST(testSimWall2D2D, reflection1 ) {
  SimWall2D wall1(1.0, 1.0, 7.0, 1.0), wall2(7.0, 1.0, 1.0, 1.0);
  Point2 init(2.0, 3.0), intersection(4.0, 1.0);
  Rot2 actual1 = wall1.reflection(init, intersection);
  Rot2 actual2 = wall2.reflection(init, intersection);
  Rot2 expected = Rot2::fromDegrees(45);
  EXPECT(assert_equal(expected, actual1, tol));
  EXPECT(assert_equal(expected, actual2, tol));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
