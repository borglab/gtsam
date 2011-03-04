/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussianFactor.h
 * @brief   Linear Factor....A Gaussian
 * @brief   linearFactor
 * @author  Richard Roberts, Christian Potthast
 */

// \callgraph

#pragma once

#include <gtsam/inference/FactorGraph.h>
#include <gtsam/inference/IndexFactor.h>
#include <gtsam/linear/Errors.h>

#include <string>
#include <utility>

namespace gtsam {

  // Forward declarations
  class VectorValues;
  class Permutation;
  class GaussianConditional;
  template<class C> class BayesNet;

  /**
   * Base Class for a linear factor.
   * GaussianFactor is non-mutable (all methods const!).
   * The factor value is exp(-0.5*||Ax-b||^2)
   */
  class GaussianFactor: public IndexFactor {

  protected:

    /** Copy constructor */
    GaussianFactor(const This& f) : IndexFactor(f) {}

    /** Construct from derived type */
    GaussianFactor(const GaussianConditional& c);

    /** Constructor from a collection of keys */
    template<class KeyIterator> GaussianFactor(KeyIterator beginKey, KeyIterator endKey) :
        Base(beginKey, endKey) {}

    /** Default constructor for I/O */
    GaussianFactor() {}

    /** Construct unary factor */
    GaussianFactor(Index j) : IndexFactor(j) {}

    /** Construct binary factor */
    GaussianFactor(Index j1, Index j2) : IndexFactor(j1, j2) {}

    /** Construct ternary factor */
    GaussianFactor(Index j1, Index j2, Index j3) : IndexFactor(j1, j2, j3) {}

    /** Construct 4-way factor */
    GaussianFactor(Index j1, Index j2, Index j3, Index j4) : IndexFactor(j1, j2, j3, j4) {}

    /** Construct n-way factor */
    GaussianFactor(const std::set<Index>& js) : IndexFactor(js) {}


  public:

    enum SolveMethod { SOLVE_QR, SOLVE_PREFER_CHOLESKY };

    typedef GaussianConditional ConditionalType;
    typedef boost::shared_ptr<GaussianFactor> shared_ptr;

    // Implementing Testable interface
    virtual void print(const std::string& s = "") const = 0;
    virtual bool equals(const GaussianFactor& lf, double tol = 1e-9) const = 0;

    virtual double error(const VectorValues& c) const = 0; /**  0.5*(A*x-b)'*D*(A*x-b) */

    /** Return the dimension of the variable pointed to by the given key iterator */
    virtual size_t getDim(const_iterator variable) const = 0;

    /**
     * Permutes the GaussianFactor, but for efficiency requires the permutation
     * to already be inverted.  This acts just as a change-of-name for each
     * variable.  The order of the variables within the factor is not changed.
     */
    virtual void permuteWithInverse(const Permutation& inversePermutation) = 0;

    /**
     * Combine and eliminate several factors.
     */
    static std::pair<boost::shared_ptr<BayesNet<GaussianConditional> >, shared_ptr> CombineAndEliminate(
        const FactorGraph<GaussianFactor>& factors, size_t nrFrontals=1, SolveMethod solveMethod=SOLVE_QR);

  }; // GaussianFactor


  /** unnormalized error */
  template<class FACTOR>
  double gaussianError(const FactorGraph<FACTOR>& fg, const VectorValues& x) {
    double total_error = 0.;
    BOOST_FOREACH(const typename FACTOR::shared_ptr& factor, fg) {
      total_error += factor->error(x);
    }
    return total_error;
  }

  /** return A*x-b */
  template<class FACTOR>
  Errors gaussianErrors(const FactorGraph<FACTOR>& fg, const VectorValues& x) {
    return *gaussianErrors_(fg, x);
  }

  /** shared pointer version */
  template<class FACTOR>
  boost::shared_ptr<Errors> gaussianErrors_(const FactorGraph<FACTOR>& fg, const VectorValues& x) {
    boost::shared_ptr<Errors> e(new Errors);
    BOOST_FOREACH(const typename FACTOR::shared_ptr& factor, fg) {
      e->push_back(factor->error_vector(x));
    }
    return e;
  }


} // namespace gtsam
