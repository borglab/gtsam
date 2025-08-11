/**
 * Kernels for continuous interpolating Finite Impulse Response filters
 * This can be used to define continuous-time trajectory models
 */

#pragma once

#include <gtsam/slam/expressions.h>

namespace gtsam {


/**
 * A base class for convolutional kernel functions
 */
class KernelBase
{
public:
  /** evaluate the kernel function at time t */
  virtual double evaluate(const double& t, OptionalJacobian<1, 1>  H = {}) const = 0;

  /** evaluate the kernel function's Nth derivative at t */
  virtual double evaluateDerivative(size_t derivative, double t, OptionalJacobian<1, 1>  H = {}) const = 0;

  /** earliest value of t that returns a non-constant kernel value */
  virtual double getBeginning() const = 0;
  /** centre of the distribution */
  virtual double getCenter() const = 0;  
  /** latest value of t that returns a non-constant kernel value */
  virtual double getEnd() const = 0;
  /** the number of times this kernel can be sensibly differentiated */
  virtual size_t getValidDerivatives() const = 0;

  /** total length where the kernel is non-zero */
  inline double getLength() const 
  {
    return getEnd() - getBeginning();
  };

  /** default destructor */
  virtual ~KernelBase() = default;

};




} // namespace gtsam