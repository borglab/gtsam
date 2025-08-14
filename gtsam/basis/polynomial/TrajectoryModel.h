#pragma once

#include <gtsam/basis/polynomial/kernels.h> // API for the basis functions
#include <gtsam/basis/polynomial/IrwinHall.h> // provides the default kernel



/*
TrajectoryModel generates Expression<T>s that encode the sampling of interpolating funcitons like splines.
The sample coordinate (sampling time) is an Expression, allowing the user to optimise for a time of best fit within a known curve.
The control points are Expressions, allowing a model to interpolate between results of other models
The template type supports gtsam Lie Groups

This trajectory model uses a lie-algebra accumulation form described in:
https://openaccess.thecvf.com/content_CVPR_2020/papers/Sommer_Efficient_D_Computation_for_Cumulative_B-Splines_on_Lie_Groups_CVPR_2020_paper.pdf

In the special case of spines with uniformly-spaced control points ('canonical splines'),
  the basis functions reduce down to the Irwin Hall CDF.

This formulation is equivalent to a describing the trajactory as a convolution
  between the control points and the PDF.

Conceptually this convolution is like to taking a numerical derivative in the Lie Algebra,
  then encoding an integral operator as a (very) long FIR filter (the step function)
  this leaves the actual FIR kernel up to the user, as long as it it integrated at compile time.
  
  points x kernel
  points x kernel x derivative x integral
  (points x derivative) x kernel x integral
  (derivative(points) x kernel) x integral
  derivative(points) x integral(kernel)
This convolution form shows why the time-reversal gets applied to the integrated kernel 

When used with a time-reversed integral of an impulse response,
  this replicates the behaviour of a FIR filter.
  The impulse should integrate to 1.0
using an Irwin Hall probability density function as the impulse,
  replicates cardinal splines.


There is possibly a more numerically way of doing this where reference point
  is chosen as close to the control point as possible, and the logspace
  accumulation is performed both ways out from the reference point.
  The CDF would need to be evaluated and split at the reference point

*/

namespace gtsam {


/**
 * An Expression factory for cardinal splines and similar convolution-based interpolations
 * Used to model continuous systems and associate them with asynchronous measurements
 */
template <class T>
class TrajectoryModel 
{
public:

  /**
   * Constructor
   * @param density the number of control points per unit (1.0) step in sample location
   * @param kernel the interpolating kernel to use, See IrwinHallCDF.cpp for splines
   * default is IrwinHallCDF2 for cubic splines
   * @param points Expressions representing control points for the curve, defaults to empty
   * @param padFront at least `kernel.getLength()` padding control points will be used,
   *  false places padding at the back, delaying response to the control points (default)
   *  true places padding at the front, pre-empting response to the control points
   */
  TrajectoryModel(
    double density = 1.0,
    const KernelBase& kernel = kernels::IrwinHallCDF2,
    const std::vector<Expression<T>>& points = {},
    bool padFront = false
  ):
    density_(density),
    kernel_(kernel),
    points_(points),
    padFront_(padFront),
    kernelOffset_(padFront ?
        kernel.getEnd() :
        kernel.getBeginning()),
    windowPre_(padFront ?
        0 :
        ceil(kernel.getLength())),
    windowPost_(padFront ?
        ceil(kernel.getLength()) :
        0 )
  {}

public:
  const double density_;
  const KernelBase& kernel_;
private:
  std::vector<Expression<T>> points_;
public:
  const bool padFront_;
  const double kernelOffset_;
  const int windowPre_;
  const int windowPost_;
public:

  /**
   * Append control points to the trajectory
   * point can wrap Keys, constants, or other computations.
   * A 2d interpolation mesh can be constructed by using evaluations of
   *   an orthogonal TrajectoryModel as the control points.
   * @param point the control point to insert
   */
  void addControlPoint(Expression<T> point)
  {
    points_.push_back(point);
  }

  /**
  get a read-only reference to the current control points
   */
  const std::vector<Expression<T>> getControlPoints() const {
    return points_;
    }


  /**
   * Sample the trajectory model at a given timestamp
   * The return value is an expression containing the spline evalation algorithm.
   * timestamp is an Expression, allowing the user to inject solution-dependent
   * corrections before sampling the spline.
   * 
   * windowStart and windowEnd are used to limit the number of control points
   *   considered in the resulting Expression.
   * Setting windowEnd beyond `get_points.size()` will not add future points to the resulting expression.
   * Timestamps always start at zero, and control points are spaced with an interval of exactly 1.0.
   * Default window uses all available control points.
   * 
   * @param timestamp the evaluation point Expression
   * @param windowStart the earliest plausible value for timestamp,
   *   reduces the number of control points evaluated
   * @param windowEnd the latest plausible value for timestamp,
   *   reduces the number of control points evaluated.
   *   a negative value (default) sample up until the `get_points.size()-1`-th point.
   */
  Expression<T> sampleTrajectory(
    Double_ timestamp,
    double windowStart = 0,
    double windowEnd = -1) const;


  /**
   * Sample the Nth derivative of the trajectory.
   * TangentVector obeys vector-space rules, so all subsequent derivatives share the same type.
   * 
   * @param timestamp the evaluation point Expression
   * @param windowStart the earliest plausible value for timestamp,
   *   reduces the number of control points evaluated
   * @param windowEnd the latest plausible value for timestamp,
   *   reduces the number of control points evaluated.
   *   a negative value (default) sample up until the `get_points.size()-1`-th point.
   * @param derivative how many times to differentiate (1: velocity, 2: acceleration, etc)
   *    a derivative of zero is equivalent to `logmap(sampleTrajectory(windowStart-kernel.getLength()), sampleTrajectory(timestamp))`
   */
  Expression<typename traits<T>::TangentVector> sampleTrajectoryDerivative(
    Double_ timestamp,
    double windowStart = 0,
    double windowEnd = -1,
    size_t derivative = 1) const;



private:

  /**
   * kernelInterpolate evaluates the spline by collecting a subset of the
   * control points and their respective weights, then passes them to cumulativePathSum.
   * The return value is an Expression referring to timestamp, and all control points in points[start] : points[end]
   * @param kernel a reference to the kernel being sampled for weights
   * @param timestamp an Expression for the evaluation point in the resulting curve
   * @param points a reference to the vector of control points
   * @param start the index of the first control point to sample
   * @param end the index of the last control point to sample
   */
  static Expression<T> kernelInterpolate(
    const KernelBase& kernel, // the kernel to use for interpolation
    const Double_& timestamp, // the timestamp to interpolate at
    const std::vector<Expression<T>>& points, // a reference to the control points to interpolate
    size_t start, size_t end // the range of control points to interpolate
  );


  /**
   * kernelInterpolateDerivative  evaluates derivatives of the spline by
   * collecting a subset of the control points and their respective weights,
   * then passes them to cumulativePathSum.
   * The return value is an Expression referring to timestamp and all control points in points[start] : points[end]
   * @param kernel a reference to the kernel being sampled for weights
   * @param timestamp an Expression for the evaluation point in the resulting curve
   * @param points a reference to the vector of control points
   * @param start the index of the first control point to sample
   * @param end the index of the last control point to sample
   * @param derivative which derivative to evaluate (1: velocity, 2:acceleration, etc)
   */
  static Expression<typename traits<T>::TangentVector> kernelInterpolateDerivative(
    const KernelBase& kernel, // the kernel to use for interpolation
    const Double_& timestamp, // the timestamp to interpolate at
    const std::vector<Expression<T>>& points, // a reference to the control points to interpolate
    size_t start, size_t end, // the range of control points to interpolate
    size_t derivative = 1);


  /**
   * Sample the kernel at a range of points
   * timestamp is in units of samples. Be sure to divide your timestamp by the sample rate
   * start and end are indicies into (and exact timestamps) the vector of control points.
   * @param kernel a reference to the kernel being sampled
   * @param timestamp an Expression for the evaluation point in the kernel
   * @param start the offset into the kernel (subtracted from timestamp)
   * @param end generate end - start samples
   * @param derivative how many times to differentiate the kernel
   */
  static std::vector<Double_> sampleKernel(
    const KernelBase& kernel,
    const Double_& timestamp,
    size_t start, size_t end,
    size_t derivative = 0);


  /**
   * computes a geodesic weighted sum, by weighting the steps in the path
   * from the first point and performing the accumulation in the logmap.
   *
   * Weights are the integral of a normal convolutional kernel function
   * the convolutional nature of kernel requires a time-reversal.
   * interpret this a the sum here performing a numerical derivative,
   * and the kernel being pre-baked with the integral operator to compensate.
   * the earliest entries of cdfWeights should be about 1.0.
   * @param points the control points to accumulate
   * @param cdfWeights the weights for the logspace differences.
   * cdfWeights are normally drawn from a time-reversed integral of a
   * Probability Density Function, ie, a 'Cumulative Distribution Function'
  */
  static Expression<T> cumulativePathSum(
    const std::vector<Expression<T>>& points,
    const std::vector<Double_>& cdfWeights);

  /**
   * computes the logmap form of a weighted sum of differences.
   * @param points the control points to accumulate
   * @param weights the weights to apply to each logspace difference.
  */
  static Expression<typename traits<T>::TangentVector> cumulativePathSumDerivative(
    const std::vector<Expression<T>>& points,
    const std::vector<Double_>& weights);

}; // class TrajectoryModel 


/**
 * user-facing api,
 * do input sanitisation 
 * apply filter ring-down compensations for the window domain.
 * pass references to the member variables
 */
template <class T>
Expression<T> TrajectoryModel<T>::sampleTrajectory(
  Double_ timestamp,
  double windowStart,
  double windowEnd) const 
{
  // apply padding to the window
  int start = floor(windowStart * density_) - windowPre_;
  int end = ceil(windowEnd * density_) + windowPost_;
  // sanitise inputs
  if(start < 0) start = 0;
  if(windowEnd < 0 || end > points_.size()) end = points_.size();

  // apply filter delay correction
  Double_ kernelTime = (density_ * timestamp) + Double_(kernelOffset_);

  // pass to internal methods
  return kernelInterpolate(kernel_, kernelTime, points_, start, end);
}

/**
 * user-facing api,
 * do input sanitisation 
 * apply filter ring-down compensations for the window domain.
 * pass references to the member variables
 */
template <class T>
Expression<typename traits<T>::TangentVector> TrajectoryModel<T>::sampleTrajectoryDerivative(
  Double_ timestamp,
  double windowStart,
  double windowEnd,
  size_t derivative) const
{
  // apply padding to the window
  int start = floor(windowStart * density_) - windowPre_;
  int end = ceil(windowEnd * density_) + windowPost_;
  // sanitise inputs
  if(start < 0) start = 0;
  if(windowEnd < 0 || end > points_.size()) end = points_.size();

  // apply filter delay correction
  Double_ kernelTime = (density_ * timestamp) + Double_(kernelOffset_);

  // pass to internal methods
  return pow(density_, derivative) * kernelInterpolateDerivative(kernel_, kernelTime, points_, start, end, derivative);
}


/**
 * compute a weighted sum of logmap differences between adjacent points applied to the first point
 */
template <class T>
Expression<T> TrajectoryModel<T>::cumulativePathSum(
  const std::vector<Expression<T>>& points,
  const std::vector<Double_>& cdfWeights)
{
  return expmap(points[0], cumulativePathSumDerivative(points, cdfWeights));
}


template <class T>
Expression<typename traits<T>::TangentVector> TrajectoryModel<T>::cumulativePathSumDerivative(
  const std::vector<Expression<T>>& points,
  const std::vector<Double_>& weights)
{
  Expression<typename traits<T>::TangentVector> v(traits<T>::TangentVector::Zero());
  // compute logspace vectors between adjacent points,
  // scale the vectors by the weight for the relevant point
  // accumulate the logspace vector
  //  we skip the first weight; cumulativePathSum assumes it is equal to 1.0
  for (size_t i = 1; i < points.size(); i++)
  {
    v += weights[i] * logmap(points[i-1], points[i]);
  }
  return v;
}



/**
interpolate calls sampleKernel and weighted_sum to apply the kernel over subset of the control points.
  the return type is an Expression that depends on points and timestamp,
  performing an automatically differentiated kernel convolution at linearisation time.
*/
template <class T>
Expression<T> TrajectoryModel<T>::kernelInterpolate(
  const KernelBase& kernel, // the kernel to use for interpolation
  const Double_& timestamp, // the timestamp to interpolate at
  const std::vector<Expression<T>>& points, // a reference to the control points to interpolate
  size_t start, size_t end // the range of control points to interpolate
)
{
  // copy the interval of control points that we want to interpolate within
  std::vector<Expression<T>> pointsRange(points.begin()+start, points.begin()+end);
  // generate the basis functions as expressions depending on the timestamp
  std::vector<Double_> weights_range = sampleKernel(kernel, timestamp, start, end, 0);
  // apply the weighted sum
  return cumulativePathSum(pointsRange, weights_range);
}

/**
interpolate calls sampleKernel and weighted_sum to apply the kernel over subset of the control points.
  the return type is an Expression that depends on points and timestamp,
  performing an automatically differentiated kernel convolution at linearisation time.
*/
template <class T>
Expression<typename traits<T>::TangentVector> TrajectoryModel<T>::kernelInterpolateDerivative(
  const KernelBase& kernel, // the kernel to use for interpolation
  const Double_& timestamp, // the timestamp to interpolate at
  const std::vector<Expression<T>>& points, // a reference to the control points to interpolate
  size_t start, size_t end, // the range of control points to interpolate
  size_t derivative)
{
  // copy the interval of control points that we want to interpolate within
  std::vector<Expression<T>> pointsRange(points.begin()+start, points.begin()+end);
  // generate the basis functions as expressions depending on the timestamp
  std::vector<Double_> weights_range = sampleKernel(kernel, timestamp, start, end, derivative);
  // apply the weighted sum
  return cumulativePathSumDerivative(pointsRange, weights_range);
}



/**
sample the kernel at a range of points
timestamp is is in units of samples. Be sure to divide your timestamp by the sample rate
start and end are indicies into (and exact timestamps) the vector of control points.
*/
template <class T>
std::vector<Double_> TrajectoryModel<T>::sampleKernel(
  const KernelBase& kernel,
  const Double_& timestamp,
  size_t start, size_t end,
  size_t derivative)
{
  std::vector<Double_> kernelSamples;

  // expression constructor requires us to bind to the class member method
  std::function<double(const double&, OptionalJacobian<1, 1>)> keval =
    [&kernel, derivative](const double& x, OptionalJacobian<1, 1> j)
    {
      return kernel.evaluateDerivative(derivative, x, j);
    };

  for(size_t i = start; i < end; i++)
  {
    Double_ kernelTime = timestamp - Double_(double(i));
    kernelSamples.push_back(Double_(keval, kernelTime));
  }

  return kernelSamples;
}






} // namespace gtsam


