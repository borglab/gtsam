/**
 * @file testSimPolygon2D.cpp
 * @author Alex Cunningham
 */

#include <gtsam/base/TestableAssertions.h>
#include <gtsam_unstable/geometry/SimPolygon2D.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

const double tol=1e-5;

/* ************************************************************************* */
TEST(testPolygon, triangle_basic) {

  // create a triangle from points, extract landmarks/walls, check occupancy
  Point2 pA(0,0), pB(2.0, 0.0), pC(0.0, 1.0);

  // construct and extract data
  SimPolygon2D actTriangle = SimPolygon2D::createTriangle(pA, pB, pC);
  LONGS_EQUAL(3, actTriangle.size());
  EXPECT(assert_equal(pA, actTriangle.landmark(0)));
  EXPECT(assert_equal(pB, actTriangle.landmark(1)));
  EXPECT(assert_equal(pC, actTriangle.landmark(2)));

  // get walls out
  vector<SimWall2D> actWalls = actTriangle.walls();
  vector<SimWall2D> expWalls;
  expWalls.push_back(SimWall2D(pA, pB));
  expWalls.push_back(SimWall2D(pB, pC));
  expWalls.push_back(SimWall2D(pC, pA));
  EXPECT(assert_container_equal(expWalls, actWalls, tol));

  // check occupancy - used in sampling more points
  // treat as closed polygon - points along edges also count
  EXPECT(actTriangle.contains(Point2(0.25, 0.5)));
  EXPECT(actTriangle.contains(pA));
  EXPECT(actTriangle.contains(pB));
  EXPECT(actTriangle.contains(pC));
  EXPECT(actTriangle.contains(Point2(1.0, 0.0)));

  EXPECT(!actTriangle.contains(Point2(1.0, 1.0)));
  EXPECT(!actTriangle.contains(Point2(-1.0, 1.0)));
}

/* ************************************************************************* */
TEST(testPolygon, rectangle_basic) {

  // creates an axis-aligned rectangle given a lower left corner and a height and width
  double height = 3.0, width = 2.0;
  Point2 pA(1.0, 0.0), pB(3.0, 0.0), pC(3.0, 3.0), pD(1.0, 3.0);

  // construct and extract data
  SimPolygon2D actRectangle = SimPolygon2D::createRectangle(pA, height, width);
  LONGS_EQUAL(4, actRectangle.size());
  EXPECT(assert_equal(pA, actRectangle.landmark(0)));
  EXPECT(assert_equal(pB, actRectangle.landmark(1)));
  EXPECT(assert_equal(pC, actRectangle.landmark(2)));
  EXPECT(assert_equal(pD, actRectangle.landmark(3)));

  // get walls out
  vector<SimWall2D> actWalls = actRectangle.walls();
  vector<SimWall2D> expWalls;
  expWalls.push_back(SimWall2D(pA, pB));
  expWalls.push_back(SimWall2D(pB, pC));
  expWalls.push_back(SimWall2D(pC, pD));
  expWalls.push_back(SimWall2D(pD, pA));
  EXPECT(assert_container_equal(expWalls, actWalls, tol));

  // check occupancy - used in sampling more points
  // treat as closed polygon - points along edges also count
  EXPECT(actRectangle.contains(Point2(2.0, 1.0)));
  EXPECT(actRectangle.contains(pA));
  EXPECT(actRectangle.contains(pB));
  EXPECT(actRectangle.contains(pC));
  EXPECT(actRectangle.contains(pD));
  EXPECT(actRectangle.contains(Point2(1.0, 0.0)));

  EXPECT(!actRectangle.contains(Point2(0.9, 0.5)));
  EXPECT(!actRectangle.contains(Point2(-1.0, 1.0)));
}

/* ************************************************************************* */
TEST(testPolygon, triangle_generator) {
  // generate random triangles in a bounded region with no overlap
  double side_len = 10.0; // box of length 10, centered on origin
  double mean_side_len = 2.0;   // mean length of sides
  double sigma_side_len = 0.5;  // stddev for length of sides
  double min_vertex_dist = 0.4; // minimum allowable distance between vertices
  double min_side_len = 0.1;

  // initialize the random number generator for consistency
  SimPolygon2D::seedGenerator(42u);

  vector<SimPolygon2D> existing_polys;

  SimPolygon2D actual = SimPolygon2D::randomTriangle(side_len, mean_side_len, sigma_side_len,
      min_vertex_dist, min_side_len, existing_polys);

  // use a rectangle to check that it is within boundaries
  SimPolygon2D bounding_rect = SimPolygon2D::createRectangle(Point2(-5.0,-5.0), side_len, side_len);

  EXPECT(bounding_rect.contains(actual.landmark(0)));
  EXPECT(bounding_rect.contains(actual.landmark(1)));
  EXPECT(bounding_rect.contains(actual.landmark(2)));
}

/* ************************************************************************* */
TEST(testPolygon, rectangle_generator) {
  // generate random rectangles in a bounded region with no overlap
  double side_len = 10.0; // box of length 10, centered on origin
  double mean_side_len = 2.0;   // mean length of sides
  double sigma_side_len = 0.5;  // stddev for length of sides
  double min_vertex_dist = 0.4; // minimum allowable distance between vertices
  double min_side_len = 0.1;

  // initialize the random number generator for consistency
  SimPolygon2D::seedGenerator(42u);

  vector<SimPolygon2D> existing_polys;

  SimPolygon2D actual = SimPolygon2D::randomRectangle(side_len, mean_side_len, sigma_side_len,
      min_vertex_dist, min_side_len, existing_polys);

  // use a rectangle to check that it is within boundaries
  SimPolygon2D bounding_rect = SimPolygon2D::createRectangle(Point2(-5.0,-5.0), side_len, side_len);

  EXPECT(bounding_rect.contains(actual.landmark(0)));
  EXPECT(bounding_rect.contains(actual.landmark(1)));
  EXPECT(bounding_rect.contains(actual.landmark(2)));
  EXPECT(bounding_rect.contains(actual.landmark(3)));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
