/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    LinearInequality.h
 * @brief   LinearInequality derived from Base with constrained noise model
 * @date    Nov 27, 2014
 * @author  Duy-Nguyen Ta
 * @author  Ivan Dario Jimenez
 */

#pragma once

#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/VectorValues.h>

namespace gtsam {

typedef Eigen::RowVectorXd RowVector;

/**
 * This class defines a linear inequality constraint Ax-b <= 0,
 * inheriting JacobianFactor with the special Constrained noise model
 */
class LinearInequality: public JacobianFactor {
public:
  typedef LinearInequality This; ///< Typedef to this class
  typedef JacobianFactor Base; ///< Typedef to base class
  typedef std::shared_ptr<This> shared_ptr; ///< shared_ptr to this class

private:
  Key dualKey_;
  bool active_;

public:
  /** default constructor for I/O */
  LinearInequality() :
      Base(), active_(true) {
  }

  /** Conversion from HessianFactor */
  explicit LinearInequality(const HessianFactor& hf) {
    throw std::runtime_error(
        "Cannot convert HessianFactor to LinearInequality");
  }

  /** Conversion from JacobianFactor */
  explicit LinearInequality(const JacobianFactor& jf, Key dualKey) :
      Base(jf), dualKey_(dualKey), active_(true) {
    if (!jf.isConstrained()) {
      throw std::runtime_error(
          "Cannot convert an unconstrained JacobianFactor to LinearInequality");
    }

    if (jf.get_model()->dim() != 1) {
      throw std::runtime_error("Only support single-valued inequality factor!");
    }
  }

  /** Construct unary factor */
  LinearInequality(Key i1, const RowVector& A1, double b, Key dualKey) :
      Base(i1, A1, (Vector(1) << b).finished(),
          noiseModel::Constrained::All(1)), dualKey_(dualKey), active_(true) {
  }

  /** Construct binary factor */
  LinearInequality(Key i1, const RowVector& A1, Key i2, const RowVector& A2,
      double b, Key dualKey) :
      Base(i1, A1, i2, A2, (Vector(1) << b).finished(),
          noiseModel::Constrained::All(1)), dualKey_(dualKey), active_(true) {
  }

  /** Construct ternary factor */
  LinearInequality(Key i1, const RowVector& A1, Key i2, const RowVector& A2,
      Key i3, const RowVector& A3, double b, Key dualKey) :
      Base(i1, A1, i2, A2, i3, A3, (Vector(1) << b).finished(),
          noiseModel::Constrained::All(1)), dualKey_(dualKey), active_(true) {
  }

  /** Construct an n-ary factor
   * @tparam TERMS A container whose value type is std::pair<Key, Matrix>, specifying the
   *         collection of keys and matrices making up the factor.
   *         In this inequality factor, each matrix must have only one row!! */
  template<typename TERMS>
  LinearInequality(const TERMS& terms, double b, Key dualKey) :
      Base(terms, (Vector(1) << b).finished(), noiseModel::Constrained::All(1)), dualKey_(
          dualKey), active_(true) {
  }

  /** Virtual destructor */
  ~LinearInequality() override {
  }

  /** equals */
  bool equals(const GaussianFactor& lf, double tol = 1e-9) const override {
    return Base::equals(lf, tol);
  }

  /** print */
  void print(const std::string& s = "", const KeyFormatter& formatter =
      DefaultKeyFormatter) const override {
    if (active())
      Base::print(s + "  Active", formatter);
    else
      Base::print(s + "  Inactive", formatter);
  }

  /** Clone this LinearInequality */
  GaussianFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast < GaussianFactor
        > (std::make_shared < LinearInequality > (*this));
  }

  /// dual key
  Key dualKey() const {
    return dualKey_;
  }

  /// return true if this constraint is active
  bool active() const {
    return active_;
  }

  /// Make this inequality constraint active
  void activate() {
    active_ = true;
  }

  /// Make this inequality constraint inactive
  void inactivate() {
    active_ = false;
  }

  /** Special error_vector for constraints (A*x-b) */
  Vector error_vector(const VectorValues& c) const {
    return unweighted_error(c);
  }

  /** Special error for single-valued inequality constraints. */
  double error(const VectorValues& c) const override {
    return error_vector(c)[0];
  }

  /** dot product of row s with the corresponding vector in p */
  double dotProductRow(const VectorValues& p) const {
    double aTp = 0.0;
    for (const_iterator xj = begin(); xj != end(); ++xj) {
      Vector pj = p.at(*xj);
      Vector aj = getA(xj).transpose();
      aTp += aj.dot(pj);
    }
    return aTp;
  }

};
// \ LinearInequality

/// traits
template<> struct traits<LinearInequality> : public Testable<LinearInequality> {
};

} // \ namespace gtsam

