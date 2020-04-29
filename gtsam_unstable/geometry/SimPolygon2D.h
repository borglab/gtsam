/**
 * @file SimPolygon2D.h
 * @brief Polygons for simulation use
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam_unstable/geometry/SimWall2D.h>

#include <map>
#include <random>

namespace gtsam {

/**
 * General polygon class for convex polygons
 */
class GTSAM_UNSTABLE_EXPORT SimPolygon2D {
protected:
  Point2Vector landmarks_;
  static std::minstd_rand rng;

public:

  /** Don't use this constructor, use a named one instead */
  SimPolygon2D() {}

  /** seed the random number generator - only needs to be done once */
  static void seedGenerator(unsigned long seed);

  /** Named constructor for creating triangles */
  static SimPolygon2D createTriangle(const Point2& pA, const Point2& pB, const Point2& pC);

  /**
   * Named constructor for creating axis-aligned rectangles
   * @param p is the lower-left corner
   */
  static SimPolygon2D createRectangle(const Point2& p, double height, double width);

  /**
   * Randomly generate a triangle that does not conflict with others
   * Uniformly distributed over box area, with normally distributed lengths of edges
   * THROWS: std::runtime_error if can't find a position
   */
  static SimPolygon2D randomTriangle(double side_len, double mean_side_len, double sigma_side_len,
      double min_vertex_dist, double min_side_len, const std::vector<SimPolygon2D>& existing_polys);

  /**
   * Randomly generate a rectangle that does not conflict with others
   * Uniformly distributed over box area, with normally distributed lengths of edges
   * THROWS: std::runtime_error if can't find a position
   */
  static SimPolygon2D randomRectangle(double side_len, double mean_side_len, double sigma_side_len,
      double min_vertex_dist, double min_side_len, const std::vector<SimPolygon2D>& existing_polys);

  // access to underlying points
  const Point2& landmark(size_t i) const { return landmarks_[i]; }
  size_t size() const { return landmarks_.size(); }
  const Point2Vector& vertices() const { return landmarks_; }

  // testable requirements
  bool equals(const SimPolygon2D& p, double tol=1e-5) const;
  void print(const std::string& s="") const;

  /**
   * Get a set of walls along the edges
   */
  std::vector<SimWall2D> walls() const;

  /**
   * Core function for randomly generating scenarios.
   * Polygons are closed, convex shapes.
   * @return true if the given point is contained by this polygon
   */
  bool contains(const Point2& p) const;

  /**
   * Checks two polygons to determine if they overlap
   * @return true iff at least one vertex of one polygon is contained in the other
   */
  bool overlaps(const SimPolygon2D& p) const;

  /** returns true iff p is contained in any of a set of polygons */
  static bool anyContains(const Point2& p, const std::vector<SimPolygon2D>& obstacles);

  /** returns true iff polygon p overlaps with any of a set of polygons */
  static bool anyOverlaps(const SimPolygon2D& p, const std::vector<SimPolygon2D>& obstacles);

  /** returns true iff p is inside a square centered at zero with side s */
  static bool insideBox(double s, const Point2& p);

  /** returns true iff p is within threshold of any point in S */
  static bool nearExisting(const Point2Vector& S,
      const Point2& p, double threshold);

  /** pick a random point uniformly over a box of side s */
  static Point2 randomPoint2(double s);

  /** randomly generate a Rot2 with a uniform distribution over theta */
  static Rot2 randomAngle();

  /** generate a distance from a normal distribution given a mean and sigma */
  static double randomDistance(double mu, double sigma, double min_dist = -1.0);

  /** pick a random point within a box that is further than dist d away from existing landmarks */
  static Point2 randomBoundedPoint2(double boundary_size,
      const Point2Vector& landmarks, double min_landmark_dist);

  /** pick a random point within a box that meets above requirements, as well as staying out of obstacles */
  static Point2 randomBoundedPoint2(double boundary_size,
      const Point2Vector& landmarks,
      const std::vector<SimPolygon2D>& obstacles, double min_landmark_dist);

  /** pick a random point that only avoid obstacles */
  static Point2 randomBoundedPoint2(double boundary_size,
      const std::vector<SimPolygon2D>& obstacles);

  /** pick a random point in box defined by lower left and upper right corners */
  static Point2 randomBoundedPoint2(
      const Point2& LL_corner, const Point2& UR_corner,
      const Point2Vector& landmarks,
      const std::vector<SimPolygon2D>& obstacles, double min_landmark_dist);

  /** pick a random pose in a bounded area that is not in an obstacle */
  static Pose2 randomFreePose(double boundary_size, const std::vector<SimPolygon2D>& obstacles);

};

typedef std::vector<SimPolygon2D> SimPolygon2DVector;

} //\namespace gtsam

