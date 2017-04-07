/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    QP.h
 * @brief   Factor graphs of a Quadratic Programming problem
 * @date    Dec 8, 2014
 * @author  Duy-Nguyen Ta
 */

#pragma once

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam_unstable/linear/EqualityFactorGraph.h>
#include <gtsam_unstable/linear/InequalityFactorGraph.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/inference/Symbol.h>

namespace gtsam {

/**
 * Struct contains factor graphs of a Quadratic Programming problem
 */
struct QP {
  GaussianFactorGraph cost; //!< Quadratic cost factors
  EqualityFactorGraph equalities; //!< linear equality constraints: cE(x) = 0
  InequalityFactorGraph inequalities; //!< linear inequality constraints: cI(x) <= 0

private:
  mutable VariableIndex cachedCostVariableIndex_;

public:
  /** default constructor */
  QP() :
      cost(), equalities(), inequalities() {
  }

  /** constructor */
  QP(const GaussianFactorGraph& _cost,
      const EqualityFactorGraph& _linearEqualities,
      const InequalityFactorGraph& _linearInequalities) :
      cost(_cost), equalities(_linearEqualities), inequalities(
          _linearInequalities) {
  }

  /** print */
  void print(const std::string& s = "") {
    std::cout << s << std::endl;
    cost.print("Quadratic cost factors: ");
    equalities.print("Linear equality factors: ");
    inequalities.print("Linear inequality factors: ");
  }

  const VariableIndex& costVariableIndex() const {
    if (cachedCostVariableIndex_.size() == 0)
      cachedCostVariableIndex_ = VariableIndex(cost);
    return cachedCostVariableIndex_;
  }

  Vector costGradient(Key key, const VectorValues& delta) const {
    Vector g = Vector::Zero(delta.at(key).size());
    if (costVariableIndex().find(key) != costVariableIndex().end()) {
      for (size_t factorIx : costVariableIndex()[key]) {
        GaussianFactor::shared_ptr factor = cost.at(factorIx);
        g += factor->gradient(key, delta);
      }
    }
    return g;
  }

  const GaussianFactorGraph& getCost() const { return cost; }
  const EqualityFactorGraph& getEqualities() const { return equalities; }
  const InequalityFactorGraph& getInequalities() const { return inequalities; }

  void add_cost(const boost::shared_ptr<GaussianFactor>& f) {
    cost.push_back(f);
  }

  /** add a unary equality with a default key for the dual variable as '=N'
   * where N is the current number of equalities added */
  void add_equality(Key i1, const Matrix& A1, const Vector& b) {
    equalities.push_back(LinearEquality(i1, A1, b,
                                        Symbol('=', equalities.size())));
  }

  /** add a binary equality with a default key for the dual variable as '=N'
   * where N is the current number of equalities added */
  void add_equality(Key i1, const Matrix& A1, Key i2, const Matrix& A2,
                    const Vector& b) {
    equalities.push_back(LinearEquality(i1, A1, i2, A2, b,
                                        Symbol('=', equalities.size())));
  }

  /** add a ternary equality with a default key for the dual variable as '=N'
   * where N is the current number of equalities added */
  void add_equality(Key i1, const Matrix& A1, Key i2, const Matrix& A2, Key i3,
                    const Matrix& A3, const Vector& b) {
    equalities.push_back(LinearEquality(i1, A1, i2, A2, i3, A3, b,
                                        Symbol('=', equalities.size())));
  }

  /** add an n-ary equality with a default key for the dual variable as '=N'
   * where N is the current number of equalities added
   * @tparam TERMS A container whose value type is std::pair<Key, Matrix>, specifying the
   *         collection of keys and matrices making up the factor. */
  template<typename TERMS>
  void add_equality(const TERMS& terms, const Vector& b) {
    equalities.push_back(LinearEquality(terms, b,
                                        Symbol('=', equalities.size())));
  }

  /** add a unary inequality with a default key for the dual variable as '<N'
   * where N is the current number of inequalities added */
  void add_inequality(Key i1, const RowVector& A1, double b) {
    inequalities.push_back(
        LinearInequality(i1, A1, b, Symbol('<', inequalities.size())));
  }

  /** add a binary inequality with a default key for the dual variable as '<N'
   * where N is the current number of inequalities added */
  void add_inequality(Key i1, const RowVector& A1, Key i2, const RowVector& A2,
                      double b) {
    inequalities.push_back(
        LinearInequality(i1, A1, i2, A2, b, Symbol('<', inequalities.size())));
  }

  /** add a tenary inequality with a default key for the dual variable as '<N'
   * where N is the current number of inequalities added */
  void add_inequality(Key i1, const RowVector& A1, Key i2, const RowVector& A2,
                      Key i3, const RowVector& A3, double b) {
    inequalities.push_back(LinearInequality(i1, A1, i2, A2, i3, A3, b,
                                            Symbol('<', inequalities.size())));
  }

  /** add an n-ary inequality with a default key for the dual variable as '=N'
   * where N is the current number of inequalities added
   * @tparam TERMS A container whose value type is std::pair<Key, Matrix>, specifying the
   *         collection of keys and matrices making up the factor. 
   *         In this inequality factor, each matrix must have only one row!!  */
  template <typename TERMS>
  void add_inequality(const TERMS& terms, double b) {
    inequalities.push_back(
        LinearInequality(terms, b, Symbol('<', inequalities.size())));
  }
  };

} // namespace gtsam

