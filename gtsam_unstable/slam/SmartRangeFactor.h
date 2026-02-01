/**
 * @file SmartRangeFactor.h
 *
 * @brief A smart factor for range-only SLAM that does initialization and marginalization
 *
 * @date Sep 10, 2012
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam_unstable/dllexport.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/inference/Key.h>
#include <gtsam/geometry/Pose2.h>

#include <list>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>
#include <optional>

namespace gtsam {

/**
 * Smart factor for range SLAM
 * @ingroup slam
 */
class SmartRangeFactor: public NoiseModelFactor {
 protected:
  struct Circle2 {
    Circle2(const Point2& p, double r) :
        center(p), radius(r) {
    }
    Point2 center;
    double radius;
  };

  typedef SmartRangeFactor This;

  std::vector<double> measurements_;  ///< Range measurements
  double variance_;  ///< variance on noise

 public:

  // Provide access to the Matrix& version of unwhitenedError
  using NoiseModelFactor::unwhitenedError;

  /** Default constructor: don't use directly */
  SmartRangeFactor() {
  }

  /**
   * Constructor
   * @param s standard deviation of range measurement noise
   */
  explicit SmartRangeFactor(double s) :
      NoiseModelFactor(noiseModel::Isotropic::Sigma(1, s)), variance_(s * s) {
  }

  ~SmartRangeFactor() override {
  }

  /// Add a range measurement to a pose with given key.
  void addRange(Key key, double measuredRange) {
    if(std::find(keys_.begin(), keys_.end(), key) != keys_.end()) {
      throw std::invalid_argument(
          "SmartRangeFactor::addRange: adding duplicate measurement for key.");
    }
    keys_.push_back(key);
    measurements_.push_back(measuredRange);
    size_t n = keys_.size();
    // Since we add the errors, the noise variance adds
    noiseModel_ = noiseModel::Isotropic::Variance(1, n * variance_);
  }

  // Testable

  /** print */
  void print(const std::string& s = "",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    std::cout << s << "SmartRangeFactor with " << size() << " measurements\n";
    NoiseModelFactor::print(s);
  }

  /** Check if two factors are equal */
  bool equals(const NonlinearFactor& f, double tol = 1e-9) const override {
    return false;
  }

  // factor interface

  /**
   * Triangulate a point from at least three pose-range pairs
   * Checks for best pair that includes first point
   * Raise runtime_error if not well defined.
   */
  Point2 triangulate(const Values& x) const {
    // create n circles corresponding to measured range around each pose
    std::list<Circle2> circles;
    size_t n = size();
    for (size_t j = 0; j < n; j++) {
      const Pose2& pose = x.at<Pose2>(keys_[j]);
      circles.push_back(Circle2(pose.translation(), measurements_[j]));
    }

    Circle2 circle1 = circles.front();
    std::optional<Point2> best_fh;
    std::optional<Circle2> bestCircle2 = std::nullopt;  // fixes issue #38

    // loop over all circles
    for (const Circle2& it : circles) {
      // distance between circle centers.
      double d = distance2(circle1.center, it.center);
      if (d < 1e-9)
        continue;  // skip circles that are in the same location
      // Find f and h, the intersection points in normalized circles
      std::optional<Point2> fh = circleCircleIntersection(
          circle1.radius / d, it.radius / d);
      // Check if this pair is better by checking h = fh->y()
      // if h is large, the intersections are well defined.
      if (fh && (!best_fh || fh->y() > best_fh->y())) {
        best_fh = fh;
        bestCircle2 = it;
      }
    }

    // use best fh to find actual intersection points
    if (bestCircle2 && best_fh) {
      auto bestCircleCenter = bestCircle2->center;
      std::list<Point2> intersections =
          circleCircleIntersection(circle1.center, bestCircleCenter, best_fh);

      // pick winner based on other measurements
      double error1 = 0, error2 = 0;
      Point2 p1 = intersections.front(), p2 = intersections.back();
      for (const Circle2& it : circles) {
        error1 += distance2(it.center, p1);
        error2 += distance2(it.center, p2);
      }
      return (error1 < error2) ? p1 : p2;
    } else {
      throw std::runtime_error("triangulate failed");
    }
  }

  /**
   * Error function *without* the NoiseModel, \f$ z-h(x) \f$.
   */
  Vector unwhitenedError(const Values& x, OptionalMatrixVecType H = nullptr) const override {
    size_t n = size();
    if (n < 3) {
      if (H) {
        // set Jacobians to zero for n<3
        for (size_t j = 0; j < n; j++)
          (*H)[j] = Matrix::Zero(3, 1);
      }
      return Z_1x1;
    } else {
      Vector error = Z_1x1;

      // triangulate to get the optimized point
      // TODO(dellaert): Should we have a (better?) variant that does this in relative coordinates ?
      Point2 optimizedPoint = triangulate(x);

      // TODO(dellaert): triangulation should be followed by an optimization given poses
      // now evaluate the errors between predicted and measured range
      for (size_t j = 0; j < n; j++) {
        const Pose2& pose = x.at<Pose2>(keys_[j]);
        if (H)
          // also calculate 1*3 derivative for each of the n poses
          error[0] += pose.range(optimizedPoint, (*H)[j]) - measurements_[j];
        else
          error[0] += pose.range(optimizedPoint) - measurements_[j];
      }
      return error;
    }
  }

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }
};
}  // \namespace gtsam

