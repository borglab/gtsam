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

  struct Circle2 {
    Circle2(const Point2& p, double r) :
        center(p), radius(r) {
    }
    Point2 center;
    double radius;
  };

  typedef SmartRangeFactor This;

  std::vector<double> measurements_; ///< Range measurements
  double sigma_; ///< standard deviation on noise

public:

  /** Default constructor: don't use directly */
  SmartRangeFactor() {
  }

  /** standard binary constructor */
  SmartRangeFactor(double sigma) : NoiseModelFactor(noiseModel::Isotropic::Sigma(1,sigma_)), sigma_(sigma) {
  }

  virtual ~SmartRangeFactor() {
  }

  /// Add a range measurement to a pose with given key.
  void addRange(Key key, double measuredRange) {
    keys_.push_back(key);
    measurements_.push_back(measuredRange);
    noiseModel_ = noiseModel::Isotropic::Sigma(keys_.size(),sigma_);
  }

  // Testable

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
  static Point2 triangulate(const std::list<Circle2>& circles) {
    Circle2 circle1 = circles.front();
    boost::optional<Point2> best_fh;
    boost::optional<Circle2> best_circle;
    BOOST_FOREACH(const Circle2& it, circles) {
      // distance between circle centers.
      double d = circle1.center.dist(it.center);
      if (d < 1e-9)
        continue;
      boost::optional<Point2> fh = Point2::CircleCircleIntersection(
          circle1.radius / d, it.radius / d);
      if (fh && (!best_fh || fh->y() > best_fh->y())) {
        best_fh = fh;
        best_circle = it;
      }
    }
    std::list<Point2> solutions = Point2::CircleCircleIntersection(
        circle1.center, best_circle->center, best_fh);
    // TODO, pick winner based on other measurement
    return solutions.front();
  }

  /**
   * Error function *without* the NoiseModel, \f$ z-h(x) \f$.
   */
  virtual Vector unwhitenedError(const Values& x,
      boost::optional<std::vector<Matrix>&> H = boost::none) const {
    size_t n = size();
    if (H) assert(H->size()==n);
    Vector errors = zero(n);
    if (n >= 3) {
      // create n circles corresponding to measured range around each pose
      std::list<Circle2> circles;
      for (size_t j = 0; j < n; j++) {
        const Pose2& pose = x.at<Pose2>(keys_[j]);
        circles.push_back(Circle2(pose.translation(), measurements_[j]));
      }
      // triangulate to get the optimized point
      Point2 optimizedPoint = triangulate(circles);
      // now evaluate the errors between predicted and measured range
      for (size_t j = 0; j < n; j++) {
        const Pose2& pose = x.at<Pose2>(keys_[j]);
        if (H) {
          // calculate n*3 derivative for each of the n poses
          (*H)[j] = zeros(n,3);
          Matrix Hj;
          errors[j] = pose.range(optimizedPoint, Hj) - measurements_[j];
          (*H)[j].row(j) = Hj;
        }
        else
          errors[j] = pose.range(optimizedPoint) - measurements_[j];
      }
    }
    return errors;
  }

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

};

} // \namespace gtsam

