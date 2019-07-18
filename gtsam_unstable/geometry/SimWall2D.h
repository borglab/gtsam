/**
 * @file SimWall2D.h
 * @brief Implementation of walls for use with simulators
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam_unstable/dllexport.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/linear/Sampler.h>

namespace gtsam {

  /**
   * General Wall class for walls defined around unordered endpoints
   * Primarily to handle ray intersections
   */
  class GTSAM_UNSTABLE_EXPORT SimWall2D {
  protected:
    Point2 a_, b_;

  public:
    /** default constructor makes canonical wall */
    SimWall2D() : a_(1.0, 0.0), b_(1.0, 1.0) {}

    /** constructors using endpoints */
    SimWall2D(const Point2& a, const Point2& b)
      : a_(a), b_(b) {}

    SimWall2D(double ax, double ay, double bx, double by)
      : a_(ax, ay), b_(bx, by) {}

    /** required by testable */
    void print(const std::string& s="") const;
    bool equals(const SimWall2D& other, double tol=1e-9) const;

    /** access */
    Point2 a() const { return a_; }
    Point2 b() const { return b_; }

    /** scales a wall to produce a new wall */
    SimWall2D scale(double s) const { return SimWall2D(s*a_, s*b_); }

    /** geometry */
    double length() const { return distance2(a_, b_); }
    Point2 midpoint() const;

    /**
     * intersection check between two segments
     * returns true if they intersect, with the intersection
     * point in the optional second argument
     */
    bool intersects(const SimWall2D& wall, boost::optional<Point2&> pt=boost::none) const;

    /**
     * norm is a 2D point representing the norm of the wall
     */
    Point2 norm() const;

    /**
     * reflection around a point of impact with a wall from a starting (init) point
     * at a given impact point (intersection), returning the angle away from the impact
     */
    Rot2 reflection(const Point2& init, const Point2& intersection) const;

  };

  typedef std::vector<SimWall2D> SimWall2DVector;

  /// traits
  template<> struct traits<SimWall2D> : public Testable<SimWall2D> {};

  /**
   * Calculates the next pose in a trajectory constrained by walls, with noise on
   * angular drift and reflection noise
   * @param cur_pose is the pose of the robot
   * @param step_size is the size of the forward step the robot tries to take
   * @param walls is a set of walls to use for bouncing
   * @param angle_drift is a sampler for angle drift (dim=1)
   * @param reflect_noise is a sampler for scatter after hitting a wall (dim=3)
   * @return the next pose for the robot
   * NOTE: samplers cannot be const
   */
  std::pair<Pose2, bool> moveWithBounce(const Pose2& cur_pose, double step_size,
      const std::vector<SimWall2D> walls, Sampler& angle_drift,
      Sampler& reflect_noise, const Rot2& bias = Rot2());

} // \namespace gtsam
