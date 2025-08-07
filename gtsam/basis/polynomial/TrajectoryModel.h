#pragma once

#include "PiecewisePolynomial.h"



/*
This trajectory model uses a lie-algebra accumulation form described in:
https://openaccess.thecvf.com/content_CVPR_2020/papers/Sommer_Efficient_Derivative_Computation_for_Cumulative_B-Splines_on_Lie_Groups_CVPR_2020_paper.pdf

In the special case of spines with uniformly-spaced control points ('canonical splines'),
  the basis funcitons reduce down to the Irwin Hall CDF.

This formulation is equivalent to a describing the trajactory as a convolution
  between the control points and the CDF.

Conceptually this is like to taking a numerical derivative in the Lie Algebra,
  then encoding an integral as a (very) long FIR filter to recover the poses.
  this leaves the actual FIR kernel up to the user
  points x kernel
  points x kernel x derivative x integral
  (points x derivative) x kernel x integral
  (derivative(points) x kernel) x integral
  derivative(points) x integral(kernel)
This convolution form clarifies the need for a time-reversal applied to the integrated kernel 

When used with a time-reversed integral of an impulse response,
  this replicates the behaviour of a FIR filter.
using an Irwin Hall probability density function as the impulse,
  replicates cardinal splines.

*/

namespace gtsam {

template <class T>
class TrajectoryModel 
{
  const kernel_base& kernel;
  std::vector<Expression<T>> points;
public:

/**
  Sample the trajectory model at a given timestamp
  window_start and window_end are used to limit the number of control points
    considered in the resulting Expression.
*/
  Expression<T> sample_trajectory(
    Double_ timestamp,
    double window_start,
    double window_end,
  );

private:

  /**
  interpolate calls sample_kernel and weighted_sum to apply the kernel over subset of the control points.
    the return type is an Expression that depends on points and timestamp,
    performing an automatically differentiated kernel convolution at linearisation time.
  */
  Expression<T> kernel_interpolate(
    const kernel_base& kernel, // the kernel to use for interpolation
    const Double_& timestamp, // the timestamp to interpolate at
    const std::vector<Expression<T>>& points, // a reference to the control points to interpolate
    size_t start, size_t end // the range of control points to interpolate
  );


  /**
  sample the kernel at a range of points
  timestamp is is in units of samples. Be sure to divide your timestamp by the sample rate
  start and end are indicies into (and exact timestamps) the vector of control points.
  */
  std::vector<Double_> sample_kernel(
    const kernel_base& kernel,
    const Double_& timestamp,
    size_t start, size_t end);


  /**
    accumulates the log-space differences between points,
    with each log-space vector scaled by its respective weight.
  */
  Expression<T> cumulative_path_sum(
    const std::vector<Expression<T>>& points,
    const std::vector<Double_>& cdf_weights);

};



/**
  Sample the trajectory model at a given timestamp
  window_start and window_end are used to limit the number of control points
    considered in the resulting Expression.
*/
template <class T>
Expression<T> TrajectoryModel<T>::sample_trajectory(
  Double_ timestamp,
  double window_start = 0,
  double window_end = -1,
)
{
  size_t start = floor(window_start);
  size_t end = floor(window_end);
  if(window_start<0)
  {
    start = 0;
  }
  if(window_end < 0 || window_end > points.size())
  {
    end = points.size();
  }

  return kernel_interpolate(
    kernel,
    timestamp,
    points,
    start, end);

}



template <class T>
Expression<T> TrajectoryModel<T>::cumulative_path_sum(
  const std::vector<Expression<T>>& points,
  const std::vector<Double_>& cdf_weights
){
  Expression<typename traits<T>::TangentVector> v = typename traits<T>::TangentVector::Zero();
  // compute logspace vectors between adjacent points,
  // scale the vectors by the CDF based weight for the relevant point
  // accumulate the logspace path as a relative vector from the first point
  for (size_t i = 0; i < points.size()-1; i++)
  {
    v += cdf_weights[i] * logmap(points[i], points[i+1]);
  }
  // apply the logspace vector to the first point
  return expmap(points[0], v);
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

  Double_ span_time = timestamp - Double_(double(start));
  // time-reverse the cdf
  Double_ kernel_time = Double_(kernel.get_end()) - span_time
  std::vector<Double_> weights_range = kernel.sample_kernel(timestamp, start, end);
  return cumulative_path_sum(points_range, weights_range);
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
  size_t start, size_t end)
{
  // expression constructor requires us to bind to the class member method
  std::function<double(const double&, OptionalJacobian<1, 1>)> keval =
    [&](const double& x, OptionalJacobian<1, 1> j) {return kernel.evaluate(x, j);};
  std::vector<Double_> vec;
  for(size_t i = start; i < end; i++)
  {
    Double_ kernel_time =
      Double_(get_end())
      + (timestamp - Double_(double(start)))
      - Double_(double(i));
    vec.push_back(Double_(keval, kernel_time));
  }
  return vec;
}






} // namespace gtsam


