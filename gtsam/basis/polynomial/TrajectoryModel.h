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

*/

namespace gtsam {

template <class T>
class TrajectoryModel 
{
public:

  TrajectoryModel(
    const kernel_base& kernel = kernels::IrwinHallCDF2, // default to cubic spline
    const std::vector<Expression<T>>& points = {},
    bool pad_front = false
  ):
    kernel_(kernel),
    points_(points),
    kernel_offset_(
      pad_front ?
        kernel.get_end() :
        kernel.get_beginning()
    )
  {}

private:
  const kernel_base& kernel_;
  std::vector<Expression<T>> points_;
  const double kernel_offset_;
public:

  /**
  Append control points to the trajectory
  point can wrap Keys, constants, or other computations.
  A 2d interpolation mesh can be constructed by using evaluations of
    an orthogonal TrajectoryModel as the control points.
  */
  void add_control_point(Expression<T> point)
  {
    points_.push_back(point);
  }

  /**
  get a read-only reference to the current control points
   */
  const std::vector<Expression<T>> get_control_points(){
    return points_;
    }


  /**
    Sample the trajectory model at a given timestamp
    The return value is an expression containing the spline evalation algorithm.
    timestamp is an Expression, allowing the user to inject solution-dependent sample rate corrections before sampling the spline.

    window_start and window_end are used to limit the number of control points
      considered in the resulting Expression.
    Setting window_end beyond `get_points.size()` will not add future points to the resulting expression.
    Timestamps always start at zero, and control points are spaced with an interval of exactly 1.0.
    Default window uses all available control points.

  */
  Expression<T> sample_trajectory(
    Double_ timestamp,
    double window_start = 0,
    double window_end = -1);


  /**
  Sample the Nth derivative of the trajectory.
  TangentVector obeys vector-space rules, so all subsequent derivatives share the same type.
  setting derivative to zero yields the logspace difference between `sample_trajectory()` and the pose just prior to window_start
  */
  Expression<typename traits<T>::TangentVector> sample_trajectory_d(
    Double_ timestamp,
    double window_start = 0,
    double window_end = -1,
    size_t derivative = 1);



public:
//private:

  /**
  interpolate calls sample_kernel and cumulative_path_sum to apply the kernel over a subset of the control points.
    the return type is an Expression that depends on points and timestamp,
    performing an automatically differentiated kernel convolution at linearisation time.
  */
  Expression<T> kernel_interpolate(
    const kernel_base& kernel, // the kernel to use for interpolation
    const Double_& timestamp, // the timestamp to interpolate at
    const std::vector<Expression<T>>& points, // a reference to the control points to interpolate
    size_t start, size_t end // the range of control points to interpolate

  );
  Expression<typename traits<T>::TangentVector> kernel_interpolate_d(
    const kernel_base& kernel, // the kernel to use for interpolation
    const Double_& timestamp, // the timestamp to interpolate at
    const std::vector<Expression<T>>& points, // a reference to the control points to interpolate
    size_t start, size_t end, // the range of control points to interpolate
    size_t derivative = 1);
  /**
  sample the kernel at a range of points
  timestamp is is in units of samples. Be sure to divide your timestamp by the sample rate
  start and end are indicies into (and exact timestamps) the vector of control points.
  */
  std::vector<Double_> sample_kernel(
    const kernel_base& kernel,
    const Double_& timestamp,
    size_t start, size_t end,
    size_t derivative = 0);


  /**
    accumulates the log-space differences between points,
    with each log-space vector scaled by its respective weight.
  */
  Expression<T> cumulative_path_sum(
    const std::vector<Expression<T>>& points,
    const std::vector<Double_>& cdf_weights);

  /**
  logspace form of the cumulative path sum, used for derivatives
  */
  Expression<typename traits<T>::TangentVector> cumulative_path_sum_d(
    const std::vector<Expression<T>>& points,
    const std::vector<Double_>& weights);

}; // class TrajectoryModel 



template <class T>
Expression<T> TrajectoryModel<T>::sample_trajectory(
  Double_ timestamp,
  double window_start,
  double window_end)
{
  size_t start = floor(window_start);
  size_t end = floor(window_end);
  // sanitise inputs
  if(window_start<0) start = 0;
  if(window_end < 0 || window_end > points_.size()) end = points_.size();
  // pass to internal method
  return kernel_interpolate(kernel_, timestamp, points_, start, end);
}

template <class T>
Expression<typename traits<T>::TangentVector> TrajectoryModel<T>::sample_trajectory_d(
  Double_ timestamp,
  double window_start,
  double window_end,
  size_t derivative)
{
  // sanitise inputs
  size_t start = floor(window_start);
  size_t end = floor(window_end);
  if(window_start<0) start = 0;
  if(window_end < 0 || window_end > points_.size()) end = points_.size();
  return kernel_interpolate_d(kernel_, timestamp, points_, start, end, derivative);
}


/*
// compute logspace vectors between adjacent points,
// scale the vectors by the CDF based weight for the relevant point
// accumulate the logspace path as a relative vector from the first point
*/
template <class T>
Expression<T> TrajectoryModel<T>::cumulative_path_sum(
  const std::vector<Expression<T>>& points,
  const std::vector<Double_>& cdf_weights)
{
  return expmap(points[0], cumulative_path_sum_d(points, cdf_weights));
}


template <class T>
Expression<typename traits<T>::TangentVector> TrajectoryModel<T>::cumulative_path_sum_d(
  const std::vector<Expression<T>>& points,
  const std::vector<Double_>& weights)
{
  Expression<typename traits<T>::TangentVector> v(traits<T>::TangentVector::Zero());
  // compute logspace vectors between adjacent points,
  // scale the vectors by the weight for the relevant point
  // accumulate the logspace vector
  //  we skip the first weight; cumulative_path_sum assumes it is equal to 1.0
  for (size_t i = 1; i < points.size(); i++)
  {
    v += weights[i] * logmap(points[i-1], points[i]);
  }
  return v;
}



/**
interpolate calls sample_kernel and weighted_sum to apply the kernel over subset of the control points.
  the return type is an Expression that depends on points and timestamp,
  performing an automatically differentiated kernel convolution at linearisation time.
*/
template <class T>
Expression<T> TrajectoryModel<T>::kernel_interpolate(
  const kernel_base& kernel, // the kernel to use for interpolation
  const Double_& timestamp, // the timestamp to interpolate at
  const std::vector<Expression<T>>& points, // a reference to the control points to interpolate
  size_t start, size_t end // the range of control points to interpolate
)
{
  // copy the interval of control points that we want to interpolate within
  std::vector<Expression<T>> points_range(points.begin()+start, points.begin()+end);
  // generate the basis functions as expressions depending on the timestamp
  std::vector<Double_> weights_range = sample_kernel(kernel, timestamp, start, end, 0);
  // apply the weighted sum
  return cumulative_path_sum(points_range, weights_range);
}

/**
interpolate calls sample_kernel and weighted_sum to apply the kernel over subset of the control points.
  the return type is an Expression that depends on points and timestamp,
  performing an automatically differentiated kernel convolution at linearisation time.
*/
template <class T>
Expression<typename traits<T>::TangentVector> TrajectoryModel<T>::kernel_interpolate_d(
  const kernel_base& kernel, // the kernel to use for interpolation
  const Double_& timestamp, // the timestamp to interpolate at
  const std::vector<Expression<T>>& points, // a reference to the control points to interpolate
  size_t start, size_t end, // the range of control points to interpolate
  size_t derivative)
{
  // copy the interval of control points that we want to interpolate within
  std::vector<Expression<T>> points_range(points.begin()+start, points.begin()+end);
  // generate the basis functions as expressions depending on the timestamp
  std::vector<Double_> weights_range = sample_kernel(kernel, timestamp, start, end, derivative);
  // apply the weighted sum
  return cumulative_path_sum_d(points_range, weights_range);
}




/**
sample the kernel at a range of points
timestamp is is in units of samples. Be sure to divide your timestamp by the sample rate
start and end are indicies into (and exact timestamps) the vector of control points.
*/
template <class T>
std::vector<Double_> TrajectoryModel<T>::sample_kernel(
  const kernel_base& kernel,
  const Double_& timestamp,
  size_t start, size_t end,
  size_t derivative)
{
  std::vector<Double_> kernel_samples;

  // expression constructor requires us to bind to the class member method
  std::function<double(const double&, OptionalJacobian<1, 1>)> keval =
    [&kernel, derivative](const double& x, OptionalJacobian<1, 1> j) {return kernel.evaluate_d(0, x, j);};

  for(size_t i = start; i < end; i++)
  {
    Double_ kernel_time =
      Double_(kernel_offset_)
      + (timestamp - Double_(double(start)))
      - Double_(double(i));
    kernel_samples.push_back(Double_(keval, kernel_time));
  }

  return kernel_samples;
}






} // namespace gtsam


