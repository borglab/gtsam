/**
 * @file SimPolygon2D.cpp
 * @author Alex Cunningham
 */

#include <iostream>
#include <random>

#include <gtsam_unstable/geometry/SimPolygon2D.h>

namespace gtsam {

using namespace std;

const size_t max_it = 100000;
std::minstd_rand SimPolygon2D::rng(42u);

/* ************************************************************************* */
void SimPolygon2D::seedGenerator(unsigned long seed) {
  rng = std::minstd_rand(seed);
}

/* ************************************************************************* */
SimPolygon2D SimPolygon2D::createTriangle(const Point2& pA, const Point2& pB, const Point2& pC) {
  SimPolygon2D result;
  result.landmarks_.push_back(pA);
  result.landmarks_.push_back(pB);
  result.landmarks_.push_back(pC);
  return result;
}

/* ************************************************************************* */
SimPolygon2D SimPolygon2D::createRectangle(const Point2& p, double height, double width) {
  SimPolygon2D result;
  result.landmarks_.push_back(p);
  result.landmarks_.push_back(p + Point2(width, 0.0));
  result.landmarks_.push_back(p + Point2(width, height));
  result.landmarks_.push_back(p + Point2(0.0, height));
  return result;
}

/* ************************************************************************* */
bool SimPolygon2D::equals(const SimPolygon2D& p, double tol) const {
  if (p.size() != size()) return false;
  for (size_t i=0; i<size(); ++i)
    if (!traits<Point2>::Equals(landmarks_[i], p.landmarks_[i], tol))
      return false;
  return true;
}

/* ************************************************************************* */
void SimPolygon2D::print(const string& s) const {
  cout << "SimPolygon " << s << ": " << endl;
  for(const Point2& p: landmarks_)
    traits<Point2>::Print(p, "   ");
}

/* ************************************************************************* */
vector<SimWall2D> SimPolygon2D::walls() const {
  vector<SimWall2D> result;
  for (size_t i=0; i<size()-1; ++i)
    result.push_back(SimWall2D(landmarks_[i], landmarks_[i+1]));
  result.push_back(SimWall2D(landmarks_[size()-1], landmarks_[0]));
  return result;
}

/* ************************************************************************* */
bool SimPolygon2D::contains(const Point2& c) const {
  vector<SimWall2D> edges = walls();
  bool initialized = false;
  bool lastSide = false;
  for(const SimWall2D& ab: edges) {
    // compute cross product of ab and ac
    Point2 dab = ab.b() - ab.a();
    Point2 dac = c - ab.a();
    double cross = dab.x() * dac.y() - dab.y() * dac.x();
    if (std::abs(cross) < 1e-6) // check for on one of the edges
      return true;
    bool side = cross > 0;
    // save the first side found
    if (!initialized) {
      lastSide = side;
      initialized = true;
      continue;
    }

    // to be inside the polygon, point must be on the same side of all lines
    if (lastSide != side)
      return false;
  }
  return true;
}

/* ************************************************************************* */
bool SimPolygon2D::overlaps(const SimPolygon2D& p) const {
  for(const Point2& a: landmarks_)
    if (p.contains(a))
      return true;
  for(const Point2& a: p.landmarks_)
    if (contains(a))
      return true;
  return false;
}

/* ***************************************************************** */
bool SimPolygon2D::anyContains(const Point2& p, const vector<SimPolygon2D>& obstacles) {
  for(const SimPolygon2D& poly: obstacles)
    if (poly.contains(p))
      return true;
  return false;
}

/* ************************************************************************* */
bool SimPolygon2D::anyOverlaps(const SimPolygon2D& p, const vector<SimPolygon2D>& obstacles) {
  for(const SimPolygon2D& poly: obstacles)
    if (poly.overlaps(p))
      return true;
  return false;
}

/* ************************************************************************* */
SimPolygon2D SimPolygon2D::randomTriangle(
    double side_len, double mean_side_len, double sigma_side_len,
    double min_vertex_dist, double min_side_len, const vector<SimPolygon2D>& existing_polys) {
  // get the current set of landmarks
  Point2Vector lms;
  double d2 = side_len/2.0;
  lms.push_back(Point2( d2, d2));
  lms.push_back(Point2(-d2, d2));
  lms.push_back(Point2(-d2,-d2));
  lms.push_back(Point2( d2,-d2));

  for(const SimPolygon2D& poly: existing_polys)
    lms.insert(lms.begin(), poly.vertices().begin(), poly.vertices().end());

  for (size_t i=0; i<max_it; ++i) {

    // find a random pose for line AB
    Pose2 xA(randomAngle(), randomBoundedPoint2(side_len, lms, existing_polys, min_vertex_dist));

    // extend line by random dist and angle to get BC
    double dAB = randomDistance(mean_side_len, sigma_side_len, min_side_len);
    double tABC = randomAngle().theta();
    Pose2 xB = xA.retract((Vector(3) << dAB, 0.0, tABC).finished());

    // extend from B to find C
    double dBC = randomDistance(mean_side_len, sigma_side_len, min_side_len);
    Pose2 xC = xB.retract(Vector::Unit(3,0)*dBC);

    // use triangle equality to verify non-degenerate triangle
    double dAC = distance2(xA.t(), xC.t());

    // form a triangle and test if it meets requirements
    SimPolygon2D test_tri = SimPolygon2D::createTriangle(xA.t(), xB.t(), xC.t());

    // check inside walls, long enough edges, far away from landmarks
    const double thresh = mean_side_len / 2.0;
    if ((dAB + dBC + thresh > dAC) &&  // triangle inequality
        (dAB + dAC + thresh > dBC) &&
        (dAC + dBC + thresh > dAB) &&
        insideBox(side_len, test_tri.landmark(0)) &&
        insideBox(side_len, test_tri.landmark(1)) &&
        insideBox(side_len, test_tri.landmark(2)) &&
        distance2(test_tri.landmark(1), test_tri.landmark(2)) > min_side_len &&
        !nearExisting(lms, test_tri.landmark(0), min_vertex_dist) &&
        !nearExisting(lms, test_tri.landmark(1), min_vertex_dist) &&
        !nearExisting(lms, test_tri.landmark(2), min_vertex_dist) &&
        !anyOverlaps(test_tri, existing_polys)) {
      return test_tri;
    }
  }
  throw runtime_error("Could not find space for a triangle");
  return SimPolygon2D::createTriangle(Point2(99,99), Point2(99,99), Point2(99,99));
}

/* ************************************************************************* */
SimPolygon2D SimPolygon2D::randomRectangle(
    double side_len, double mean_side_len, double sigma_side_len,
    double min_vertex_dist, double min_side_len, const vector<SimPolygon2D>& existing_polys) {
  // get the current set of landmarks
  Point2Vector lms;
  double d2 = side_len/2.0;
  lms.push_back(Point2( d2, d2));
  lms.push_back(Point2(-d2, d2));
  lms.push_back(Point2(-d2,-d2));
  lms.push_back(Point2( d2,-d2));
  for(const SimPolygon2D& poly: existing_polys)
    lms.insert(lms.begin(), poly.vertices().begin(), poly.vertices().end());

  const Point2 lower_corner(-side_len,-side_len);
  const Point2 upper_corner( side_len, side_len);

  for (size_t i=0; i<max_it; ++i) {

    // pick height and width to be viable distances
    double height = randomDistance(mean_side_len, sigma_side_len, min_side_len);
    double width = randomDistance(mean_side_len, sigma_side_len, min_side_len);

    // find a starting point - limited to region viable for this height/width
    Point2 pA = randomBoundedPoint2(lower_corner, upper_corner - Point2(width, height),
        lms, existing_polys, min_vertex_dist);

    // verify
    SimPolygon2D rect = SimPolygon2D::createRectangle(pA, height, width);

    // check inside walls, long enough edges, far away from landmarks
    if (insideBox(side_len, rect.landmark(0)) &&
        insideBox(side_len, rect.landmark(1)) &&
        insideBox(side_len, rect.landmark(2)) &&
        insideBox(side_len, rect.landmark(3)) &&
        !nearExisting(lms, rect.landmark(0), min_vertex_dist) &&
        !nearExisting(lms, rect.landmark(1), min_vertex_dist) &&
        !nearExisting(lms, rect.landmark(2), min_vertex_dist) &&
        !nearExisting(lms, rect.landmark(3), min_vertex_dist) &&
        !anyOverlaps(rect, existing_polys)) {
      return rect;
    }
  }
  throw runtime_error("Could not find space for a rectangle");
  return SimPolygon2D::createRectangle(Point2(99,99), 100, 100);
}

/* ***************************************************************** */
Point2 SimPolygon2D::randomPoint2(double s) {
  std::uniform_real_distribution<> gen_t(-s/2.0, s/2.0);
  return Point2(gen_t(rng), gen_t(rng));
}

/* ***************************************************************** */
Rot2 SimPolygon2D::randomAngle() {
   // modified range to avoid degenerate cases in triangles:
  std::uniform_real_distribution<> gen_r(-M_PI, M_PI);
  return Rot2::fromAngle(gen_r(rng));
}

/* ***************************************************************** */
double SimPolygon2D::randomDistance(double mu, double sigma, double min_dist) {
  std::normal_distribution<> norm_dist(mu, sigma);
  double d = -10.0;
  for (size_t i=0; i<max_it; ++i) {
    d = std::abs(norm_dist(rng));
    if (d > min_dist)
      return d;
  }
  cout << "Non viable distance: " << d << " with mu = " << mu << " sigma = " << sigma
       << " min_dist = " << min_dist << endl;
  throw runtime_error("Failed to find a viable distance");
  return std::abs(norm_dist(rng));
}

/* ***************************************************************** */
Point2 SimPolygon2D::randomBoundedPoint2(double boundary_size,
      const vector<SimPolygon2D>& obstacles) {
  for (size_t i=0; i<max_it; ++i) {
    Point2 p = randomPoint2(boundary_size);
    if (!anyContains(p, obstacles))
      return p;
  }
  throw runtime_error("Failed to find a place for a landmark!");
  return Point2(0,0);
}

/* ***************************************************************** */
Point2 SimPolygon2D::randomBoundedPoint2(double boundary_size,
    const Point2Vector& landmarks, double min_landmark_dist) {
  for (size_t i=0; i<max_it; ++i) {
    Point2 p = randomPoint2(boundary_size);
    if (!nearExisting(landmarks, p, min_landmark_dist))
      return p;
  }
  throw runtime_error("Failed to find a place for a landmark!");
  return Point2(0,0);
}

/* ***************************************************************** */
Point2 SimPolygon2D::randomBoundedPoint2(double boundary_size,
    const Point2Vector& landmarks,
    const vector<SimPolygon2D>& obstacles, double min_landmark_dist) {
  for (size_t i=0; i<max_it; ++i) {
    Point2 p = randomPoint2(boundary_size);
    if (!nearExisting(landmarks, p, min_landmark_dist) && !anyContains(p, obstacles))
      return p;
  }
  throw runtime_error("Failed to find a place for a landmark!");
  return Point2(0,0);
}

/* ***************************************************************** */
Point2 SimPolygon2D::randomBoundedPoint2(
    const Point2& LL_corner, const Point2& UR_corner,
    const Point2Vector& landmarks,
    const std::vector<SimPolygon2D>& obstacles, double min_landmark_dist) {

  std::uniform_real_distribution<> gen_x(0.0, UR_corner.x() - LL_corner.x());
  std::uniform_real_distribution<> gen_y(0.0, UR_corner.y() - LL_corner.y());

  for (size_t i=0; i<max_it; ++i) {
    Point2 p = Point2(gen_x(rng), gen_y(rng)) + LL_corner;
    if (!nearExisting(landmarks, p, min_landmark_dist) && !anyContains(p, obstacles))
      return p;
  }
  throw runtime_error("Failed to find a place for a landmark!");
  return Point2(0,0);
}

/* ***************************************************************** */
Pose2 SimPolygon2D::randomFreePose(double boundary_size, const vector<SimPolygon2D>& obstacles) {
  return Pose2(randomAngle(), randomBoundedPoint2(boundary_size, obstacles));
}

/* ***************************************************************** */
bool SimPolygon2D::insideBox(double s, const Point2& p) {
  return std::abs(p.x()) < s/2.0 && std::abs(p.y()) < s/2.0;
}

/* ***************************************************************** */
bool SimPolygon2D::nearExisting(const Point2Vector& S,
    const Point2& p, double threshold) {
  for(const Point2& Sp: S)
    if (distance2(Sp, p) < threshold)
      return true;
  return false;
}

} //\namespace gtsam

