/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file KernelBase.h
 * @brief Continuous kernels for convolution-based trajectory models.
 * @author Brett Downing
 */

#pragma once

#include <gtsam/base/OptionalJacobian.h>

#include <cstddef>

namespace gtsam {

/** Interface for a compactly supported continuous convolution kernel. */
class KernelBase {
 public:
  /** Evaluate the kernel at `t`, optionally returning its first derivative. */
  virtual double evaluate(double t, OptionalJacobian<1, 1> H = {}) const = 0;

  /**
   * Evaluate the requested derivative at `t`, optionally returning the next
   * derivative.
   */
  virtual double evaluateDerivative(size_t derivative, double t,
                                    OptionalJacobian<1, 1> H = {}) const = 0;

  /// Earliest argument at which the kernel is non-constant.
  virtual double getBeginning() const = 0;

  /// Center of the kernel support.
  virtual double getCenter() const = 0;

  /// Latest argument at which the kernel is non-constant.
  virtual double getEnd() const = 0;

  /// Highest derivative order represented by the kernel.
  virtual size_t getValidDerivatives() const = 0;

  /// Width of the kernel support.
  double getLength() const { return getEnd() - getBeginning(); }

  /// Destroy the polymorphic kernel interface.
  virtual ~KernelBase() = default;
};

}  // namespace gtsam
