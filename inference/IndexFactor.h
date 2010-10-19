/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    IndexFactor.h
 * @brief   
 * @author  Richard Roberts
 * @created Oct 17, 2010
 */

#pragma once

#include <gtsam/inference/IndexConditional.h>
#include <gtsam/inference/Factor.h>

namespace gtsam {

class IndexConditional;

class IndexFactor : public FactorBase<Index> {

public:

  typedef IndexFactor This;
  typedef FactorBase<Index> Base;
  typedef IndexConditional Conditional;
  typedef boost::shared_ptr<IndexFactor> shared_ptr;

  /** Copy constructor */
  IndexFactor(const This& f) : Base(static_cast<const Base&>(f)) {}

  /** Construct from derived type */
  IndexFactor(const IndexConditional& c);

  /** Constructor from a collection of keys */
  template<class KeyIterator> IndexFactor(KeyIterator beginKey, KeyIterator endKey) :
      Base(beginKey, endKey) {}

  /** Default constructor for I/O */
  IndexFactor() {}

  /** Construct unary factor */
  IndexFactor(Index j) : Base(j) {}

  /** Construct binary factor */
  IndexFactor(Index j1, Index j2) : Base(j1, j2) {}

  /** Construct ternary factor */
  IndexFactor(Index j1, Index j2, Index j3) : Base(j1, j2, j3) {}

  /** Construct 4-way factor */
  IndexFactor(Index j1, Index j2, Index j3, Index j4) : Base(j1, j2, j3, j4) {}

  /** Create a combined joint factor (new style for EliminationTree). */
  static shared_ptr
  Combine(const FactorGraph<This>& factors, const FastMap<Index, std::vector<Index> >& variableSlots);

  template<class FactorGraphType, class VariableIndexStorage>
  static shared_ptr Combine(const FactorGraphType& factorGraph,
      const VariableIndex<VariableIndexStorage>& variableIndex, const std::vector<size_t>& factors,
      const std::vector<Index>& variables, const std::vector<std::vector<size_t> >& variablePositions) {
    return Base::Combine<This>(factorGraph, variableIndex, factors, variables, variablePositions); }

  /**
   * eliminate the first variable involved in this factor
   * @return a conditional on the eliminated variable
   */
  boost::shared_ptr<Conditional> eliminateFirst();

  /**
   * eliminate the first nrFrontals frontal variables.
   */
  boost::shared_ptr<BayesNet<Conditional> > eliminate(size_t nrFrontals = 1);

};

}
