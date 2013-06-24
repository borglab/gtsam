/**
 * @file SmartRangeFactor.h
 *
 * @brief A smart factor for range-only SLAM that does initialization and marginalization
 * 
 * @date Sep 10, 2012
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam_unstable/base/dllexport.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Key.h>
#include <gtsam/geometry/Pose2.h>
#include <boost/foreach.hpp>
#include <map>

namespace gtsam {

/**
 * Smart factor for range SLAM
 * @addtogroup SLAM
 */
class GTSAM_UNSTABLE_EXPORT SmartRangeFactor: public NoiseModelFactor {
protected:

  struct KeyedRange {
    KeyedRange(Key k, double r) :
        key(k), range(r) {
    }
    Key key;
    double range;
  };

  struct Circle2 {
    Circle2(const Point2& p, double r) :
        center(p), radius(r) {
    }
    Point2 center;
    double radius;
  };

  /// Range measurements
  std::list<KeyedRange> measurements_;

public:

  /** Default constructor: don't use directly */
  SmartRangeFactor() {
  }

  /** standard binary constructor */
  SmartRangeFactor(const SharedNoiseModel& model) {
  }

  virtual ~SmartRangeFactor() {
  }

  /// Add a range measurement to a pose with given key.
  void addRange(Key key, double measuredRange) {
    measurements_.push_back(KeyedRange(key, measuredRange));
  }

  /// Number of measurements added
  size_t nrMeasurements() const {
    return measurements_.size();
  }

  // testable

  /** print */
  virtual void print(const std::string& s = "",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
  }

  /** Check if two factors are equal */
  virtual bool equals(const NonlinearFactor& f, double tol = 1e-9) const {
    return false;
  }

  // factor interface

  /**
   * Triangulate a point from at least three pose-range pairs
   * Checks for best pair that includes first point
   */
  static Point2 triangulate(
      const std::list<Circle2>& circles) {
    Circle2 circle1 = circles.front();
    boost::optional<Point2> best_fh;
    boost::optional<Circle2> best_circle;
    BOOST_FOREACH(const Circle2& it, circles) {
      // distance between circle centers.
      double d = circle1.center.dist(it.center);
      if (d<1e-9) continue;
      boost::optional<Point2> fh = Point2::CircleCircleIntersection(circle1.radius/d,it.radius/d);
      if (fh && (!best_fh || fh->y()>best_fh->y())) {
        best_fh = fh;
        best_circle = it;
      }
    }
    std::list<Point2> solutions = Point2::CircleCircleIntersection(
        circle1.center, best_circle->center, best_fh);
    solutions.front().print("front");
    solutions.back().print("back");
    return solutions.front();
  }

  /**
   * Error function *without* the NoiseModel, \f$ z-h(x) \f$.
   */
  virtual Vector unwhitenedError(const Values& x,
      boost::optional<std::vector<Matrix>&> H = boost::none) const {
    size_t K = nrMeasurements();
    Vector errors = zero(K);
    if (K >= 3) {
      std::list<Circle2> circles;
      BOOST_FOREACH(const KeyedRange& it, measurements_) {
        const Pose2& pose = x.at<Pose2>(it.key);
        circles.push_back(
            Circle2(pose.translation(), it.range));
      }
      Point2 optimizedPoint = triangulate(circles);
      size_t i = 0;
      BOOST_FOREACH(const Circle2& it, circles) {
        errors[i++] = it.radius - it.center.distance(optimizedPoint);
      }
    }
    return errors;
  }

};

} // \namespace gtsam

