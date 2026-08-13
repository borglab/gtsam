/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file CumulativeSplineTrajectory.h
 * @brief Expression-based cumulative splines on Lie groups.
 * @author Brett Downing
 */

#pragma once

#include <gtsam/basis/IrwinHall.h>
#include <gtsam/basis/KernelBase.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/expressions.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace gtsam {

/**
 * Factory for expressions that sample a cumulative spline on a Lie group.
 *
 * For control points @f$T_0,\ldots,T_{N-1}@f$, define consecutive tangent
 * increments
 * @f$\xi_i=\operatorname{Log}(T_{i-1}^{-1}T_i)@f$. The sampled curve is
 *
 * @f[
 * T(t)=T_0\operatorname{Exp}\left(\sum_{i=1}^{N-1}c_i(t)\xi_i\right),
 * @f]
 *
 * where each @f$c_i@f$ is a shifted cumulative kernel. This is the cumulative
 * Lie-group spline formulation of Sommer et al., "Efficient Derivative
 * Computation for Cumulative B-Splines on Lie Groups" (CVPR 2020).
 * Time-reversed Irwin-Hall CDF kernels produce uniform cardinal splines.
 * Control points and the sample coordinate are expressions, so both may be
 * optimized in a factor graph. The optional sample window keeps the resulting
 * expression sparse by excluding control points outside the kernel support.
 *
 * @tparam T Lie-group value represented by each control point.
 */
template <class T>
class CumulativeSplineTrajectory {
 public:
  /// Tangent-space value associated with one trajectory sample.
  using TangentVector = typename traits<T>::TangentVector;
  /// Expression whose value is a trajectory tangent vector.
  using TangentExpression = Expression<TangentVector>;

  /**
   * Construct a trajectory model.
   *
   * @param density Number of control points per unit sample coordinate.
   * @param kernel Integrated interpolation kernel. The default produces a
   * cubic cardinal spline.
   * @param points Initial control-point expressions.
   * @param padFront If true, place the kernel support before each control
   * point; otherwise place it after the point.
   */
  explicit CumulativeSplineTrajectory(
      double density = 1.0, const KernelBase& kernel = kernels::IrwinHallCDF2,
      const std::vector<Expression<T>>& points = {}, bool padFront = false)
      : density_(density),
        kernel_(kernel),
        points_(points),
        padFront_(padFront),
        kernelOffset_(padFront ? kernel.getEnd() : kernel.getBeginning()),
        windowPre_(padFront ? 0
                            : static_cast<int>(std::ceil(kernel.getLength()))),
        windowPost_(padFront ? static_cast<int>(std::ceil(kernel.getLength()))
                             : 0) {
    if (density <= 0.0) {
      throw std::invalid_argument(
          "CumulativeSplineTrajectory density must be positive");
    }
  }

  /** Construct a trajectory with the default cubic kernel and padding mode. */
  CumulativeSplineTrajectory(double density, bool padFront)
      : CumulativeSplineTrajectory(density, kernels::IrwinHallCDF2, {},
                                   padFront) {}

  /// Number of control points per unit sample coordinate.
  double density() const { return density_; }

  /// Kernel used to interpolate the trajectory.
  const KernelBase& kernel() const { return kernel_; }

  /// Whether kernel support is padded before each control point.
  bool padFront() const { return padFront_; }

  /** Append a key, constant, or computed expression as a control point. */
  void addControlPoint(const Expression<T>& point) { points_.push_back(point); }

  /** Append a constant value as a control point. */
  void addControlPoint(const T& point) { points_.emplace_back(point); }

  /// Read-only access to the control-point expressions.
  const std::vector<Expression<T>>& getControlPoints() const { return points_; }

  /**
   * Create an expression that samples the trajectory.
   *
   * `windowStart` and `windowEnd` bound the plausible sample coordinate and
   * prune unrelated control points from the expression. A negative
   * `windowEnd` includes all remaining control points.
   */
  Expression<T> sampleTrajectory(const Double_& timestamp,
                                 double windowStart = 0.0,
                                 double windowEnd = -1.0) const {
    const auto [start, end] = controlPointWindow(windowStart, windowEnd);
    const Double_ kernelTime = density_ * timestamp + Double_(kernelOffset_);
    return kernelInterpolate(kernel_, kernelTime, points_, start, end);
  }

  /**
   * Sample a constant-control trajectory at a numeric timestamp.
   *
   * This value-based overload evaluates the same expression tree as the
   * expression API and is convenient for language wrappers and visualization.
   * Every control point must be constant; keyed expressions require the
   * expression overload and a `Values` assignment.
   */
  T sampleTrajectory(double timestamp, double windowStart = 0.0,
                     double windowEnd = -1.0) const {
    return sampleTrajectory(Double_(timestamp), windowStart, windowEnd)
        .value(Values());
  }

  /**
   * Create an expression for a trajectory derivative in the tangent space.
   *
   * @param derivative Derivative order: 1 for velocity, 2 for acceleration,
   * and so on. Zero returns the cumulative tangent displacement.
   */
  TangentExpression sampleTrajectoryDerivative(const Double_& timestamp,
                                               double windowStart = 0.0,
                                               double windowEnd = -1.0,
                                               size_t derivative = 1) const {
    const auto [start, end] = controlPointWindow(windowStart, windowEnd);
    const Double_ kernelTime = density_ * timestamp + Double_(kernelOffset_);
    return Double_(std::pow(density_, derivative)) *
           kernelInterpolateDerivative(kernel_, kernelTime, points_, start, end,
                                       derivative);
  }

  /**
   * Evaluate a tangent derivative for constant controls and numeric time.
   *
   * The result is the derivative of the accumulated tangent coordinate. It is
   * not a body-frame velocity without the appropriate Lie-group Jacobian.
   */
  TangentVector sampleTrajectoryDerivative(double timestamp,
                                           double windowStart = 0.0,
                                           double windowEnd = -1.0,
                                           size_t derivative = 1) const {
    return sampleTrajectoryDerivative(Double_(timestamp), windowStart,
                                      windowEnd, derivative)
        .value(Values());
  }

 private:
  /// Convert a time window into a nonempty range of control-point indices.
  std::pair<size_t, size_t> controlPointWindow(double windowStart,
                                               double windowEnd) const {
    if (points_.empty()) {
      throw std::invalid_argument(
          "CumulativeSplineTrajectory requires at least one control point");
    }

    int start =
        static_cast<int>(std::floor(windowStart * density_)) - windowPre_;
    int end =
        windowEnd < 0.0
            ? static_cast<int>(points_.size())
            : static_cast<int>(std::ceil(windowEnd * density_)) + windowPost_;
    start = std::max(0, start);
    end = std::min(end, static_cast<int>(points_.size()));
    if (start >= end) {
      throw std::invalid_argument(
          "CumulativeSplineTrajectory sample window contains no control "
          "points");
    }
    return {static_cast<size_t>(start), static_cast<size_t>(end)};
  }

  /// Interpolate a group-valued sample from a bounded control-point range.
  static Expression<T> kernelInterpolate(
      const KernelBase& kernel, const Double_& timestamp,
      const std::vector<Expression<T>>& points, size_t start, size_t end) {
    const std::vector<Expression<T>> pointRange(points.begin() + start,
                                                points.begin() + end);
    const std::vector<Double_> weights =
        sampleKernel(kernel, timestamp, start, end, 0);
    return cumulativePathSum(pointRange, weights);
  }

  /// Interpolate a tangent derivative from a bounded control-point range.
  static TangentExpression kernelInterpolateDerivative(
      const KernelBase& kernel, const Double_& timestamp,
      const std::vector<Expression<T>>& points, size_t start, size_t end,
      size_t derivative) {
    const std::vector<Expression<T>> pointRange(points.begin() + start,
                                                points.begin() + end);
    const std::vector<Double_> weights =
        sampleKernel(kernel, timestamp, start, end, derivative);
    return cumulativePathSumDerivative(pointRange, weights);
  }

  /// Build expression-valued shifted-kernel samples for the selected range.
  static std::vector<Double_> sampleKernel(const KernelBase& kernel,
                                           const Double_& timestamp,
                                           size_t start, size_t end,
                                           size_t derivative) {
    std::vector<Double_> samples;
    samples.reserve(end - start);
    const std::function<double(const double&, OptionalJacobian<1, 1>)>
        evaluateKernel =
            [&kernel, derivative](const double& x, OptionalJacobian<1, 1> H) {
              return kernel.evaluateDerivative(derivative, x, H);
            };

    for (size_t index = start; index < end; ++index) {
      const Double_ kernelTime =
          timestamp - Double_(static_cast<double>(index));
      samples.emplace_back(evaluateKernel, kernelTime);
    }
    return samples;
  }

  /// Accumulate weighted increments and retract them from the first point.
  static Expression<T> cumulativePathSum(
      const std::vector<Expression<T>>& points,
      const std::vector<Double_>& cumulativeWeights) {
    return expmap(points.front(),
                  cumulativePathSumDerivative(points, cumulativeWeights));
  }

  /// Form the weighted sum of consecutive Logmap increments.
  static TangentExpression cumulativePathSumDerivative(
      const std::vector<Expression<T>>& points,
      const std::vector<Double_>& weights) {
    TangentExpression tangent(TangentVector::Zero());
    for (size_t index = 1; index < points.size(); ++index) {
      tangent += weights[index] * logmap(points[index - 1], points[index]);
    }
    return tangent;
  }

  const double density_;               ///< Control points per time unit.
  const KernelBase& kernel_;           ///< Non-owning interpolation kernel.
  std::vector<Expression<T>> points_;  ///< Control-point expressions.
  const bool padFront_;                ///< Direction of support padding.
  const double kernelOffset_;          ///< Kernel-to-time origin shift.
  const int windowPre_;                ///< Control points before a window.
  const int windowPost_;               ///< Control points after a window.
};

}  // namespace gtsam
