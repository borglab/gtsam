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

#include <gtsam/inference/FactorGraph.h>
#include <gtsam/linear/Errors.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/GaussianBayesNet.h>

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
    typedef GaussianBayesNet bayesnet_type;
    typedef GaussianVariableIndex<> variableindex_type;

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
    	push_back(sharedFactor(new GaussianFactor(b)));
  	}

  	/** Add a unary factor */
    void add(Index key1, const Matrix& A1,
  			const Vector& b, const SharedDiagonal& model) {
    	push_back(sharedFactor(new GaussianFactor(key1,A1,b,model)));
  	}

  	/** Add a binary factor */
    void add(Index key1, const Matrix& A1,
  			Index key2, const Matrix& A2,
  			const Vector& b, const SharedDiagonal& model) {
    	push_back(sharedFactor(new GaussianFactor(key1,A1,key2,A2,b,model)));
  	}

  	/** Add a ternary factor */
    void add(Index key1, const Matrix& A1,
  			Index key2, const Matrix& A2,
  			Index key3, const Matrix& A3,
  			const Vector& b, const SharedDiagonal& model) {
    	push_back(sharedFactor(new GaussianFactor(key1,A1,key2,A2,key3,A3,b,model)));
  	}

  	/** Add an n-ary factor */
    void add(const std::vector<std::pair<Index, Matrix> > &terms,
  	    const Vector &b, const SharedDiagonal& model) {
    	push_back(sharedFactor(new GaussianFactor(terms,b,model)));
  	}

    /**
     * Return the set of variables involved in the factors (computes a set
     * union).
     */
    typedef std::set<Index, std::less<Index>, boost::fast_pool_allocator<Index> > Keys;
    Keys keys() const;

    /** Permute the variables in the factors */
    void permuteWithInverse(const Permutation& inversePermutation);

		/** return A*x-b */
		Errors errors(const VectorValues& x) const;

		/** shared pointer version */
		boost::shared_ptr<Errors> errors_(const VectorValues& x) const;

			/** unnormalized error */
		double error(const VectorValues& x) const;

		/** return A*x */
		Errors operator*(const VectorValues& x) const;

		/* In-place version e <- A*x that overwrites e. */
		void multiplyInPlace(const VectorValues& x, Errors& e) const;

		/* In-place version e <- A*x that takes an iterator. */
		void multiplyInPlace(const VectorValues& x, const Errors::iterator& e) const;

//		/** return A^e */
//		VectorValues operator^(const Errors& e) const;

		/** x += alpha*A'*e */
		void transposeMultiplyAdd(double alpha, const Errors& e, VectorValues& x) const;

		/**
		 * Calculate Gradient of A^(A*x-b) for a given config
		 * @param x: VectorValues specifying where to calculate gradient
		 * @return gradient, as a VectorValues as well
		 */
		VectorValues gradient(const VectorValues& x) const;

		/** Unnormalized probability. O(n) */
		double probPrime(const VectorValues& c) const {
			return exp(-0.5 * error(c));
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
     * Add zero-mean i.i.d. Gaussian prior terms to each variable
     * @param sigma Standard deviation of Gaussian
     */
    GaussianFactorGraph add_priors(double sigma, const GaussianVariableIndex<>& variableIndex) const;
    GaussianFactorGraph add_priors(double sigma) const;


  };


  /* ************************************************************************* */
  template<class VARIABLEINDEXSTORAGE>
  class GaussianVariableIndex : public VariableIndex<VARIABLEINDEXSTORAGE> {
  public:
    typedef VariableIndex<VARIABLEINDEXSTORAGE> Base;
    typedef typename VARIABLEINDEXSTORAGE::template type_factory<size_t>::type storage_type;

    storage_type dims_;

  public:
    typedef boost::shared_ptr<GaussianVariableIndex> shared_ptr;

    /** Construct an empty GaussianVariableIndex */
    GaussianVariableIndex() {}

    /**
     * Constructor from a GaussianFactorGraph, lets the base class build the
     * column-wise index then fills the dims_ array.
     */
    GaussianVariableIndex(const GaussianFactorGraph& factorGraph);

    /**
     * Constructor to "upgrade" from the base class without recomputing the
     * column index, i.e. just fills the dims_ array.
     */
    GaussianVariableIndex(const VariableIndex<VARIABLEINDEXSTORAGE>& variableIndex, const GaussianFactorGraph& factorGraph);

    /**
     * Another constructor to upgrade from the base class using an existing
     * array of variable dimensions.
     */
    GaussianVariableIndex(const VariableIndex<VARIABLEINDEXSTORAGE>& variableIndex, const storage_type& dimensions);

    const storage_type& dims() const { return dims_; }
    size_t dim(Index variable) const { Base::checkVar(variable); return dims_[variable]; }

    /** Permute */
    void permute(const Permutation& permutation);

    /** Augment this variable index with the contents of another one */
    void augment(const GaussianFactorGraph& factorGraph);

  protected:
    GaussianVariableIndex(size_t nVars) : Base(nVars), dims_(nVars) {}
    void fillDims(const GaussianFactorGraph& factorGraph);
  };


  /* ************************************************************************* */
  template<class STORAGE>
  GaussianVariableIndex<STORAGE>::GaussianVariableIndex(const GaussianFactorGraph& factorGraph) :
  Base(factorGraph), dims_(Base::index_.size()) {
    fillDims(factorGraph); }

  /* ************************************************************************* */
  template<class STORAGE>
  GaussianVariableIndex<STORAGE>::GaussianVariableIndex(
      const VariableIndex<STORAGE>& variableIndex, const GaussianFactorGraph& factorGraph) :
      Base(variableIndex), dims_(Base::index_.size()) {
    fillDims(factorGraph); }

  /* ************************************************************************* */
  template<class STORAGE>
  GaussianVariableIndex<STORAGE>::GaussianVariableIndex(
      const VariableIndex<STORAGE>& variableIndex, const storage_type& dimensions) :
      Base(variableIndex), dims_(dimensions) {
    assert(Base::index_.size() == dims_.size()); }

  /* ************************************************************************* */
  template<class STORAGE>
  void GaussianVariableIndex<STORAGE>::fillDims(const GaussianFactorGraph& factorGraph) {
    // Store dimensions of each variable
    assert(dims_.size() == Base::index_.size());
    for(Index var=0; var<Base::index_.size(); ++var)
      if(!Base::index_[var].empty()) {
        size_t factorIndex = Base::operator [](var).front().factorIndex;
        size_t variablePosition = Base::operator [](var).front().variablePosition;
        boost::shared_ptr<const GaussianFactor> factor(factorGraph[factorIndex]);
        dims_[var] = factor->getDim(factor->begin() + variablePosition);
      } else
        dims_[var] = 0;
  }

  /* ************************************************************************* */
  template<class STORAGE>
  void GaussianVariableIndex<STORAGE>::permute(const Permutation& permutation) {
    VariableIndex<STORAGE>::permute(permutation);
    storage_type original(this->dims_.size());
    this->dims_.swap(original);
    for(Index j=0; j<permutation.size(); ++j)
      this->dims_[j] = original[permutation[j]];
  }

  /* ************************************************************************* */
  template<class STORAGE>
  void GaussianVariableIndex<STORAGE>::augment(const GaussianFactorGraph& factorGraph) {
    Base::augment(factorGraph);
    dims_.resize(Base::index_.size(), 0);
    BOOST_FOREACH(boost::shared_ptr<const GaussianFactor> factor, factorGraph) {
      for(GaussianFactor::const_iterator var=factor->begin(); var!=factor->end(); ++var) {
#ifndef NDEBUG
        if(dims_[*var] != 0)
          assert(dims_[*var] == factor->getDim(var));
#endif
        if(dims_[*var] == 0)
          dims_[*var] = factor->getDim(var);
      }
    }
//    for(Index var=0; var<dims_.size(); ++var) {
//#ifndef NDEBUG
//      if(var >= varIndex.dims_.size() || varIndex.dims_[var] == 0)
//        assert(dims_[var] != 0);
//      else if(varIndex.dims_[var] != 0 && dims_[var] != 0)
//        assert(dims_[var] == varIndex.dims_[var]);
//#endif
//      if(dims_[var] == 0)
//        dims_[var] = varIndex.dims_[var];
//    }
  }

//	/**
//	 * Returns the augmented matrix version of a set of factors
//	 * with the corresponding noiseModel
//	 * @param factors is the set of factors to combine
//	 * @param ordering of variables needed for matrix column order
//	 * @return the augmented matrix and a noise model
//	 */
//	template <class FACTORS>
//	std::pair<Matrix, SharedDiagonal> combineFactorsAndCreateMatrix(
//			const FACTORS& factors,
//			const Ordering& order, const Dimensions& dimensions);

} // namespace gtsam
