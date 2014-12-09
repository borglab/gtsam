/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * LinearInequality.h
 * @brief: LinearInequality derived from Base with constrained noise model
 * @date: Nov 27, 2014
 * @author: thduynguyen
 */

#pragma once

#include <gtsam/linear/JacobianFactor.h>

namespace gtsam {

/**
 * This class defines Linear constraints by inherit Base
 * with the special Constrained noise model
 */
class LinearInequality: public JacobianFactor {
public:
  typedef LinearInequality This; ///< Typedef to this class
  typedef JacobianFactor Base; ///< Typedef to base class
  typedef boost::shared_ptr<This> shared_ptr; ///< shared_ptr to this class

private:
  Key dualKey_;

public:
  /** default constructor for I/O */
  LinearInequality() :
      Base() {
  }

  /** Conversion from HessianFactor (does Cholesky to obtain Jacobian matrix) */
  explicit LinearInequality(const HessianFactor& hf) {
    throw std::runtime_error(
        "Cannot convert HessianFactor to LinearInequality");
  }

  /** Construct unary factor */
  LinearInequality(Key i1, const Matrix& A1, const Vector& b, Key dualKey) :
      Base(i1, A1, b, noiseModel::Constrained::All(b.rows())), dualKey_(dualKey) {
  }

  /** Construct binary factor */
  LinearInequality(Key i1, const Matrix& A1, Key i2, const Matrix& A2,
      const Vector& b, Key dualKey) :
      Base(i1, A1, i2, A2, b, noiseModel::Constrained::All(b.rows())), dualKey_(
          dualKey) {
  }

  /** Construct ternary factor */
  LinearInequality(Key i1, const Matrix& A1, Key i2, const Matrix& A2, Key i3,
      const Matrix& A3, const Vector& b, Key dualKey) :
      Base(i1, A1, i2, A2, i3, A3, b, noiseModel::Constrained::All(b.rows())), dualKey_(
          dualKey) {
  }

  /** Construct four-ary factor */
  LinearInequality(Key i1, const Matrix& A1, Key i2, const Matrix& A2, Key i3,
      const Matrix& A3, Key i4, const Matrix& A4, const Vector& b, Key dualKey) :
      Base(i1, A1, i2, A2, i3, A3, i4, A4, b,
          noiseModel::Constrained::All(b.rows())), dualKey_(dualKey) {
  }

  /** Construct five-ary factor */
  LinearInequality(Key i1, const Matrix& A1, Key i2, const Matrix& A2, Key i3,
      const Matrix& A3, Key i4, const Matrix& A4, Key i5, const Matrix& A5,
      const Vector& b, Key dualKey) :
      Base(i1, A1, i2, A2, i3, A3, i4, A4, i5, A5, b,
          noiseModel::Constrained::All(b.rows())), dualKey_(dualKey) {
  }

  /** Construct six-ary factor */
  LinearInequality(Key i1, const Matrix& A1, Key i2, const Matrix& A2, Key i3,
      const Matrix& A3, Key i4, const Matrix& A4, Key i5, const Matrix& A5,
      Key i6, const Matrix& A6, const Vector& b, Key dualKey) :
      Base(i1, A1, i2, A2, i3, A3, i4, A4, i5, A5, i6, A6, b,
          noiseModel::Constrained::All(b.rows())), dualKey_(dualKey) {
  }

  /** Construct an n-ary factor
   * @tparam TERMS A container whose value type is std::pair<Key, Matrix>, specifying the
   *         collection of keys and matrices making up the factor. */
  template<typename TERMS>
  LinearInequality(const TERMS& terms, const Vector& b, Key dualKey) :
      Base(terms, b, noiseModel::Constrained::All(b.rows())), dualKey_(dualKey) {
  }

  /** Virtual destructor */
  virtual ~LinearInequality() {
  }

  /** equals */
  virtual bool equals(const GaussianFactor& lf, double tol = 1e-9) const {
    return Base::equals(lf, tol);
  }

  /** print */
  virtual void print(const std::string& s = "", const KeyFormatter& formatter =
      DefaultKeyFormatter) const {
    Base::print(s, formatter);
  }

  /** Clone this LinearInequality */
  virtual GaussianFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<GaussianFactor>(
        boost::make_shared<LinearInequality>(*this));
  }

  /// dual key
  Key dualKey() const { return dualKey_; }

  /** Special error_vector for constraints (A*x-b) */
  Vector error_vector(const VectorValues& c) const {
    return unweighted_error(c);
  }

  /** Special error for constraints.
   * I think it should be zero, as this function is meant for objective cost.
   * But the name "error" can be misleading.
   * TODO: confirm with Frank!! */
  virtual double error(const VectorValues& c) const {
    return 0.0;
  }

  /** dot product of row s with the corresponding vector in p */
  double dotProductRow(size_t s, const VectorValues& p) const {
    double ajTp = 0.0;
    for (const_iterator xj = begin(); xj != end(); ++xj) {
      Vector pj = p.at(*xj);
      Vector aj = getA(xj).row(s);
      ajTp += aj.dot(pj);
    }
    return ajTp;
  }

};
// LinearInequality

}// gtsam

