/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussianFactorGraph.h
 * @brief   Linear Factor Graph where all factors are Gaussians
 * @author  Kai Ni
 * @author  Christian Potthast
 * @author  Alireza Fathi
 */ 

#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/tuple/tuple.hpp>

#include <gtsam/base/FastSet.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/linear/Errors.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/SharedDiagonal.h>

namespace gtsam {

  /**
   * A Linear Factor Graph is a factor graph where all factors are Gaussian, i.e.
   *   Factor == GaussianFactor
   *   VectorValues = A values structure of vectors
   * Most of the time, linear factor graphs arise by linearizing a non-linear factor graph.
   */
  class GaussianFactorGraph : public FactorGraph<GaussianFactor> {
  public:

    typedef boost::shared_ptr<GaussianFactorGraph> shared_ptr;

    /**
     * Default constructor 
     */
    GaussianFactorGraph() {}

    /**
     * Constructor that receives a Chordal Bayes Net and returns a GaussianFactorGraph
     */
    GaussianFactorGraph(const GaussianBayesNet& CBN);

    /** Constructor from a factor graph of GaussianFactor or a derived type */
    template<class DERIVEDFACTOR>
    GaussianFactorGraph(const FactorGraph<DERIVEDFACTOR>& fg) {
      push_back(fg);
    }

    /** Add a null factor */
    void add(const Vector& b) {
      push_back(sharedFactor(new JacobianFactor(b)));
    }

    /** Add a unary factor */
    void add(Index key1, const Matrix& A1,
        const Vector& b, const SharedDiagonal& model) {
      push_back(sharedFactor(new JacobianFactor(key1,A1,b,model)));
    }

    /** Add a binary factor */
    void add(Index key1, const Matrix& A1,
        Index key2, const Matrix& A2,
        const Vector& b, const SharedDiagonal& model) {
      push_back(sharedFactor(new JacobianFactor(key1,A1,key2,A2,b,model)));
    }

    /** Add a ternary factor */
    void add(Index key1, const Matrix& A1,
        Index key2, const Matrix& A2,
        Index key3, const Matrix& A3,
        const Vector& b, const SharedDiagonal& model) {
      push_back(sharedFactor(new JacobianFactor(key1,A1,key2,A2,key3,A3,b,model)));
    }

    /** Add an n-ary factor */
    void add(const std::vector<std::pair<Index, Matrix> > &terms,
        const Vector &b, const SharedDiagonal& model) {
      push_back(sharedFactor(new JacobianFactor(terms,b,model)));
    }

    /**
     * Return the set of variables involved in the factors (computes a set
     * union).
     */
    typedef FastSet<Index> Keys;
    Keys keys() const;

    /** Permute the variables in the factors */
    void permuteWithInverse(const Permutation& inversePermutation);

    /** Unnormalized probability. O(n) */
    double probPrime(const VectorValues& c) const {
      return exp(-0.5 * gaussianError(*this, c));
    }

    /**
     * static function that combines two factor graphs
     * @param const &lfg1 Linear factor graph
     * @param const &lfg2 Linear factor graph
     * @return a new combined factor graph
     */
    static GaussianFactorGraph combine2(const GaussianFactorGraph& lfg1,
        const GaussianFactorGraph& lfg2);

    /**
     * combine two factor graphs
     * @param *lfg Linear factor graph
     */
    void combine(const GaussianFactorGraph &lfg);

    /**
     * Return vector of i, j, and s to generate an m-by-n sparse Jacobain matrix
     * such that S(i(k),j(k)) = s(k), which can be given to MATLAB's sparse.
     * The standard deviations are baked into A and b
     * @param first column index for each variable
     */
    std::vector<boost::tuple<size_t,size_t,double> > sparseJacobian(const std::vector<size_t>& columnIndices) const;

    // get b
//    void getb(VectorValues &b) const ;
//    VectorValues getb() const ;
//
//    // allocate a vectorvalues of b's structure
//    VectorValues allocateVectorValuesb() const ;

  };

} // namespace gtsam
