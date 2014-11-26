/*
 * LPSolver.h
 * @brief:
 * @date: May 1, 2014
 * @author: Duy-Nguyen Ta
 */

#pragma once

#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/3rdparty/lp_solve_5.5/lp_lib.h>

#include <boost/range/irange.hpp>

namespace gtsam {

/**
 * Class to solve a LP problem, using lpsolve
 * TODO: This class might currently be inefficient due to lots of memory copy.
 * Consider making lp a class variable and support setConstraints to allow to reuse
 * this class and avoid building meta information every time.
 */
class LPSolver {
  VectorValues objectiveCoeffs_;
  GaussianFactorGraph::shared_ptr constraints_;
  VectorValues lowerBounds_, upperBounds_;
  std::map<Key, size_t> variableColumnNo_, variableDims_;
  size_t nrColumns_;
  KeySet freeVars_;

public:
  /** Constructor with optional lower/upper bounds
   * Note that lpsolve by default enforces a 0.0 lower bound and no upper bound on each variable, i.e. x>=0
   * We do NOT adopt this convention here. If no lower/upper bounds are specified, the variable will be
   * set as unbounded, i.e. -inf <= x <= inf.
   */
  LPSolver(const VectorValues& objectiveCoeffs,
      const GaussianFactorGraph::shared_ptr& constraints,
      const VectorValues& lowerBounds = VectorValues(),
      const VectorValues& upperBounds = VectorValues()) :
      objectiveCoeffs_(objectiveCoeffs), constraints_(constraints), lowerBounds_(
          lowerBounds), upperBounds_(upperBounds) {
    buildMetaInformation();
  }

  /**
   * Build data structures to support converting between gtsam and lpsolve
   * TODO: consider lp as a class variable and support setConstraints
   * to avoid rebuild this meta data
   */
  void buildMetaInformation();

  /// Get functions for unittest checking
  const std::map<Key, size_t>& variableColumnNo() const {
    return variableColumnNo_;
  }
  const std::map<Key, size_t>& variableDims() const {
    return variableDims_;
  }
  size_t nrColumns() const {
    return nrColumns_;
  }
  const KeySet& freeVars() const {
    return freeVars_;
  }

  /**
   * Build lpsolve's column number for a list of keys
   */
  template<class KEYLIST>
  std::vector<int> buildColumnNo(const KEYLIST& keyList) const {
    std::vector<int> columnNo;
    BOOST_FOREACH(Key key, keyList) {
      std::vector<int> varIndices = boost::copy_range<std::vector<int> >(
          boost::irange(variableColumnNo_.at(key),
              variableColumnNo_.at(key) + variableDims_.at(key)));
      columnNo.insert(columnNo.end(), varIndices.begin(), varIndices.end());
    }
    return columnNo;
  }

  /// Add all [scalar] constraints in a constrained Jacobian factor to lp
  void addConstraints(const boost::shared_ptr<lprec>& lp,
      const JacobianFactor::shared_ptr& jacobian) const;

  /**
   * Add all bounds to lp.
   * Note: lp by default enforces a 0.0 lower bound and no upper bound on each variable, i.e. x>=0
   * We do NOT adopt this convention here. If no lower/upper bounds are specified, the variable will be
   * set as unbounded, i.e. -inf <= x <= inf.
   */
  void addBounds(const boost::shared_ptr<lprec>& lp) const;

  /**
   * Main function to build lpsolve model
   * TODO: consider lp as a class variable and support setConstraints
   * to avoid rebuild meta data
   */
  boost::shared_ptr<lprec> buildModel() const;

  /// Convert lpsolve result back to gtsam's VectorValues
  VectorValues convertToVectorValues(REAL* row) const;

  /// Solve
  VectorValues solve() const;
};

} /* namespace gtsam */
