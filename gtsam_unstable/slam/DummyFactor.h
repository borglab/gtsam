/**
 * @file DummyFactor.h
 *
 * @brief A simple factor that can be used to trick gtsam solvers into believing a graph is connected.
 *
 * @date Sep 10, 2012
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam_unstable/dllexport.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

class GTSAM_UNSTABLE_EXPORT DummyFactor : public NonlinearFactor {
protected:

  // Store the dimensions of the variables and the dimension of the full system
  std::vector<size_t> dims_;
  size_t rowDim_; ///< choose dimension for the rows

public:

  /** Default constructor: don't use directly */
  DummyFactor() : rowDim_(1) { }

  /** standard binary constructor */
  DummyFactor(const Key& key1, size_t dim1, const Key& key2, size_t dim2);

  ~DummyFactor() override {}

  // testable

  /** print */
  void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override;

  /** Check if two factors are equal */
  bool equals(const NonlinearFactor& f, double tol = 1e-9) const override;

  // access

  const std::vector<size_t>& dims() const { return dims_; }

  // factor interface

  /**
   * Calculate the error of the factor - zero for dummy factors
   */
  double error(const Values& c) const override { return 0.0; }

  /** get the dimension of the factor (number of rows on linearization) */
  size_t dim() const override { return rowDim_; }

  /** linearize to a GaussianFactor */
  std::shared_ptr<GaussianFactor> linearize(const Values& c) const override;

  /**
   * Creates a shared_ptr clone of the factor - needs to be specialized to allow
   * for subclasses
   *
   * By default, throws exception if subclass does not implement the function.
   */
  NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<NonlinearFactor>(
        NonlinearFactor::shared_ptr(new DummyFactor(*this)));
  }

};

} // \namespace gtsam




