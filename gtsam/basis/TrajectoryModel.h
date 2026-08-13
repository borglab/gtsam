/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file TrajectoryModel.h
 * @brief Expression-based cumulative splines on Lie groups.
 * @author Brett Downing
 */

#pragma once

#include <gtsam/basis/IrwinHall.h>
#include <gtsam/basis/Kernel.h>
#include <gtsam/slam/expressions.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace gtsam {

/**
 * Factory for expressions that sample a convolution-based continuous
 * trajectory.
 *
 * The implementation uses the cumulative Lie-group spline formulation from
 * Sommer et al., "Efficient Derivative Computation for Cumulative B-Splines
 * on Lie Groups" (CVPR 2020). Control points and the sample coordinate are
 * expressions, so both can participate in optimization. Uniform cardinal
 * splines are obtained with the time-reversed Irwin-Hall CDF kernels declared
 * in IrwinHall.h.
 *
 * @tparam T Lie-group value represented by each control point.
 */
template <class T>
class TrajectoryModel {
 public:
  using TangentVector = typename traits<T>::TangentVector;
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
  explicit TrajectoryModel(double density = 1.0,
                           const KernelBase& kernel = kernels::IrwinHallCDF2,
                           const std::vector<Expression<T>>& points = {},
                           bool padFront = false)
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
      throw std::invalid_argument("TrajectoryModel density must be positive");
    }
  }

  /// Number of control points per unit sample coordinate.
  double density() const { return density_; }

  /// Kernel used to interpolate the trajectory.
  const KernelBase& kernel() const { return kernel_; }

  /// Whether kernel support is padded before each control point.
  bool padFront() const { return padFront_; }

  /** Append a key, constant, or computed expression as a control point. */
  void addControlPoint(const Expression<T>& point) { points_.push_back(point); }

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

 private:
  std::pair<size_t, size_t> controlPointWindow(double windowStart,
                                               double windowEnd) const {
    if (points_.empty()) {
      throw std::invalid_argument(
          "TrajectoryModel requires at least one control point");
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
          "TrajectoryModel sample window contains no control points");
    }
    return {static_cast<size_t>(start), static_cast<size_t>(end)};
  }

  static Expression<T> kernelInterpolate(
      const KernelBase& kernel, const Double_& timestamp,
      const std::vector<Expression<T>>& points, size_t start, size_t end) {
    const std::vector<Expression<T>> pointRange(points.begin() + start,
                                                points.begin() + end);
    const std::vector<Double_> weights =
        sampleKernel(kernel, timestamp, start, end, 0);
    return cumulativePathSum(pointRange, weights);
  }

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

  static Expression<T> cumulativePathSum(
      const std::vector<Expression<T>>& points,
      const std::vector<Double_>& cumulativeWeights) {
    return expmap(points.front(),
                  cumulativePathSumDerivative(points, cumulativeWeights));
  }

  static TangentExpression cumulativePathSumDerivative(
      const std::vector<Expression<T>>& points,
      const std::vector<Double_>& weights) {
    TangentExpression tangent(TangentVector::Zero());
    for (size_t index = 1; index < points.size(); ++index) {
      tangent += weights[index] * logmap(points[index - 1], points[index]);
    }
    return tangent;
  }

  const double density_;
  const KernelBase& kernel_;
  std::vector<Expression<T>> points_;
  const bool padFront_;
  const double kernelOffset_;
  const int windowPre_;
  const int windowPost_;
};

}  // namespace gtsam
