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
  virtual double evaluate(const double& t, OptionalJacobian<1, 1>  H = {}) const = 0;
  // earliest value of t that returns a non-zero kernel value
  virtual double get_beginning() const = 0;
  // centre of the distribution
  virtual double get_center() const = 0;  
  // last value of t that returns a non-zero kernel value
  virtual double get_end() const = 0;
  

  virtual ~kernel_base() = default;


  // generates a vector of Expressions that will sample the kernel's basis function at `timestamp` during optimisation.
  std::vector<Double_> sample_kernel(
    const Double_& timestamp,
    size_t start, size_t end
  );


};




} // namespace gtsam