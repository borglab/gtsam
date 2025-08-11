/**
 * Kernels for continuous interpolating Finite Impulse Response filters
 * This can be used to define continuous-time trajectory models
 */

#pragma once

#include <gtsam/slam/expressions.h>

namespace gtsam {


class kernel_base
{
public:

  /**
  evaluate the kernel function at t
   */
  virtual double evaluate(const double& t, OptionalJacobian<1, 1>  H = {}) const = 0;
  /**
  evaluate the kernel function's nth derivative at t
   */
  virtual double evaluate_d(size_t derivative, double t, OptionalJacobian<1, 1>  H = {}) const = 0;

  // earliest value of t that returns a non-zero kernel value
  virtual double get_beginning() const = 0;
  // centre of the distribution
  virtual double get_center() const = 0;  
  // last value of t that returns a non-zero kernel value
  virtual double get_end() const = 0;
  virtual size_t get_valid_derivatives() const = 0;

  inline double get_length() const 
  {
    return get_end() - get_beginning();
  };


  virtual ~kernel_base() = default;

};




} // namespace gtsam