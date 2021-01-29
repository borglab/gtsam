/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    LinearEquality.h
 * @brief   LinearEquality derived from Base with constrained noise model
 * @date    Nov 27, 2014
 * @author  Duy-Nguyen Ta
 */

#pragma once

#include <gtsam/linear/JacobianFactor.h>

namespace gtsam {

/**
 * This class defines a linear equality constraints, inheriting JacobianFactor
 * with the special Constrained noise model
 */
class LinearEquality: public JacobianFactor {
public:
  typedef LinearEquality This; ///< Typedef to this class
  typedef JacobianFactor Base; ///< Typedef to base class
  typedef boost::shared_ptr<This> shared_ptr; ///< shared_ptr to this class

private:
  Key dualKey_;

public:
  /** default constructor for I/O */
  LinearEquality() :
      Base() {
  }

  /**
   * Construct from a constrained noisemodel JacobianFactor with a dual key.
   */
  explicit LinearEquality(const JacobianFactor& jf, Key dualKey) :
      Base(jf), dualKey_(dualKey) {
    if (!jf.isConstrained()) {
      throw std::runtime_error(
          "Cannot convert an unconstrained JacobianFactor to LinearEquality");
    }
  }

  /** Conversion from HessianFactor (does Cholesky to obtain Jacobian matrix) */
  explicit LinearEquality(const HessianFactor& hf) {
    throw std::runtime_error("Cannot convert HessianFactor to LinearEquality");
  }

  /** Construct unary factor */
  LinearEquality(Key i1, const Matrix& A1, const Vector& b, Key dualKey) :
      Base(i1, A1, b, noiseModel::Constrained::All(b.rows())), dualKey_(dualKey) {
  }

  /** Construct binary factor */
  LinearEquality(Key i1, const Matrix& A1, Key i2, const Matrix& A2,
      const Vector& b, Key dualKey) :
      Base(i1, A1, i2, A2, b, noiseModel::Constrained::All(b.rows())), dualKey_(
          dualKey) {
  }

  /** Construct ternary factor */
  LinearEquality(Key i1, const Matrix& A1, Key i2, const Matrix& A2, Key i3,
      const Matrix& A3, const Vector& b, Key dualKey) :
      Base(i1, A1, i2, A2, i3, A3, b, noiseModel::Constrained::All(b.rows())), dualKey_(
          dualKey) {
  }

  /** Construct an n-ary factor
   * @tparam TERMS A container whose value type is std::pair<Key, Matrix>, specifying the
   *         collection of keys and matrices making up the factor. */
  template<typename TERMS>
  LinearEquality(const TERMS& terms, const Vector& b, Key dualKey) :
      Base(terms, b, noiseModel::Constrained::All(b.rows())), dualKey_(dualKey) {
  }

  /** Virtual destructor */
  ~LinearEquality() override {
  }

  /** equals */
  bool equals(const GaussianFactor& lf, double tol = 1e-9) const override {
    return Base::equals(lf, tol);
  }

  /** print */
  void print(const std::string& s = "", const KeyFormatter& formatter =
      DefaultKeyFormatter) const override {
    Base::print(s, formatter);
  }

  /** Clone this LinearEquality */
  GaussianFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast < GaussianFactor
        > (boost::make_shared < LinearEquality > (*this));
  }

  /// dual key
  Key dualKey() const {
    return dualKey_;
  }

  /// for active set method: equality constraints are always active
  bool active() const {
    return true;
  }

  /** Special error_vector for constraints (A*x-b) */
  Vector error_vector(const VectorValues& c) const {
    return unweighted_error(c);
  }

  /** Special error for constraints.
   * I think it should be zero, as this function is meant for objective cost.
   * But the name "error" can be misleading.
   * TODO: confirm with Frank!! */
  double error(const VectorValues& c) const override {
    return 0.0;
  }

};
// \ LinearEquality

/// traits
template<> struct traits<LinearEquality> : public Testable<LinearEquality> {
};

} // \ namespace gtsam

