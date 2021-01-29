/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    LinearCost.h
 * @brief   LinearCost derived from JacobianFactor to support linear cost functions c'x
 * @date    Nov 27, 2014
 * @author  Duy-Nguyen Ta
 */

#pragma once

#include <gtsam/linear/JacobianFactor.h>

namespace gtsam {

typedef Eigen::RowVectorXd RowVector;

/**
 * This class defines a linear cost function c'x
 * which is a JacobianFactor with only one row
 */
class LinearCost: public JacobianFactor {
public:
  typedef LinearCost This; ///< Typedef to this class
  typedef JacobianFactor Base; ///< Typedef to base class
  typedef boost::shared_ptr<This> shared_ptr; ///< shared_ptr to this class

public:
  /** default constructor for I/O */
  LinearCost() :
      Base() {
  }

  /** Conversion from HessianFactor */
  explicit LinearCost(const HessianFactor& hf) {
    throw std::runtime_error("Cannot convert HessianFactor to LinearCost");
  }

  /** Conversion from JacobianFactor */
  explicit LinearCost(const JacobianFactor& jf) :
      Base(jf) {
    if (jf.isConstrained()) {
      throw std::runtime_error(
          "Cannot convert a constrained JacobianFactor to LinearCost");
    }

    if (jf.get_model()->dim() != 1) {
      throw std::runtime_error(
          "Only support single-valued linear cost factor!");
    }
  }

  /** Construct unary factor */
  LinearCost(Key i1, const RowVector& A1) :
      Base(i1, A1, Vector1::Zero()) {
  }

  /** Construct binary factor */
  LinearCost(Key i1, const RowVector& A1, Key i2, const RowVector& A2, double b) :
      Base(i1, A1, i2, A2, Vector1::Zero()) {
  }

  /** Construct ternary factor */
  LinearCost(Key i1, const RowVector& A1, Key i2, const RowVector& A2, Key i3,
      const RowVector& A3) :
      Base(i1, A1, i2, A2, i3, A3, Vector1::Zero()) {
  }

  /** Construct an n-ary factor
   * @tparam TERMS A container whose value type is std::pair<Key, Matrix>, specifying the
   *         collection of keys and matrices making up the factor. */
  template<typename TERMS>
  LinearCost(const TERMS& terms) :
      Base(terms, Vector1::Zero()) {
  }

  /** Virtual destructor */
  ~LinearCost() override {
  }

  /** equals */
  bool equals(const GaussianFactor& lf, double tol = 1e-9) const override {
    return Base::equals(lf, tol);
  }

  /** print */
  void print(const std::string& s = "", const KeyFormatter& formatter =
      DefaultKeyFormatter) const override {
    Base::print(s + " LinearCost: ", formatter);
  }

  /** Clone this LinearCost */
  GaussianFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast < GaussianFactor
        > (boost::make_shared < LinearCost > (*this));
  }

  /** Special error_vector for constraints (A*x-b) */
  Vector error_vector(const VectorValues& c) const {
    return unweighted_error(c);
  }

  /** Special error for single-valued inequality constraints. */
  double error(const VectorValues& c) const override {
    return error_vector(c)[0];
  }
};
// \ LinearCost

/// traits
template<> struct traits<LinearCost> : public Testable<LinearCost> {
};

} // \ namespace gtsam

