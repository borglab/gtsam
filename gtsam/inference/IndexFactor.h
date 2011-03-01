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

#include <gtsam/inference/FactorBase.h>

namespace gtsam {

  // Forward declaration of IndexConditional
  class IndexConditional;

  /**
   * IndexFactor serves two purposes.  It is the base class for all linear
   * factors (GaussianFactor, JacobianFactor, HessianFactor), and also
   * functions as a symbolic factor with Index keys, used to do symbolic
   * elimination by JunctionTree.
   *
   * This class provides symbolic elimination, via the CombineAndEliminate
   * function.  Combine and eliminate can also be called separately, but for
   * this and derived classes calling them separately generally does extra
   * work.
   *
   * It derives from FactorBase with a key type of Index, which is an unsigned
   * integer.
   */
  class IndexFactor : public FactorBase<Index> {

  public:

    typedef IndexFactor This;
    typedef FactorBase<Index> Base;

    /** Elimination produces an IndexConditional */
    typedef IndexConditional ConditionalType;

    /** Overriding the shared_ptr typedef */
    typedef boost::shared_ptr<IndexFactor> shared_ptr;

    /** Copy constructor */
    IndexFactor(const This& f) : Base(f) {}

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

    /** Construct n-way factor */
    IndexFactor(const std::set<Index>& js) : Base(js) {}

    /**
     * Combine and eliminate several factors.
     */
    static std::pair<BayesNet<ConditionalType>::shared_ptr, shared_ptr> CombineAndEliminate(
        const FactorGraph<This>& factors, size_t nrFrontals=1);

    /** Create a combined joint factor (new style for EliminationTree). */
    static shared_ptr
    Combine(const FactorGraph<This>& factors, const FastMap<Index, std::vector<Index> >& variableSlots);

    /**
     * eliminate the first variable involved in this factor
     * @return a conditional on the eliminated variable
     */
    boost::shared_ptr<ConditionalType> eliminateFirst();

    /**
     * eliminate the first nrFrontals frontal variables.
     */
    boost::shared_ptr<BayesNet<ConditionalType> > eliminate(size_t nrFrontals = 1);

  };

}
