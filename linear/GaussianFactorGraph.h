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

  	/** Add a null factor */
    inline void add(const Vector& b) {
    	push_back(sharedFactor(new GaussianFactor(b)));
  	}

  	/** Add a unary factor */
    inline void add(Index key1, const Matrix& A1,
  			const Vector& b, const SharedDiagonal& model) {
    	push_back(sharedFactor(new GaussianFactor(key1,A1,b,model)));
  	}

  	/** Add a binary factor */
    inline void add(Index key1, const Matrix& A1,
  			Index key2, const Matrix& A2,
  			const Vector& b, const SharedDiagonal& model) {
    	push_back(sharedFactor(new GaussianFactor(key1,A1,key2,A2,b,model)));
  	}

  	/** Add a ternary factor */
    inline void add(Index key1, const Matrix& A1,
  			Index key2, const Matrix& A2,
  			Index key3, const Matrix& A3,
  			const Vector& b, const SharedDiagonal& model) {
    	push_back(sharedFactor(new GaussianFactor(key1,A1,key2,A2,key3,A3,b,model)));
  	}

  	/** Add an n-ary factor */
    inline void add(const std::vector<std::pair<Index, Matrix> > &terms,
  	    const Vector &b, const SharedDiagonal& model) {
    	push_back(sharedFactor(new GaussianFactor(terms,b,model)));
  	}

    /**
     * Return the set of variables involved in the factors (computes a set
     * union).
     */
    std::set<Index, std::less<Index>, boost::fast_pool_allocator<Index> > keys() const;

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

//  	/**
//  	 * Calculate Gradient of A^(A*x-b) for a given config
//  	 * @param x: VectorValues specifying where to calculate gradient
//  	 * @return gradient, as a VectorValues as well
//  	 */
//  	VectorValues gradient(const VectorValues& x) const;

		/** Unnormalized probability. O(n) */
		double probPrime(const VectorValues& c) const {
			return exp(-0.5 * error(c));
		}

//    /**
//     * find the separator, i.e. all the nodes that have at least one
//     * common factor with the given node. FD: not used AFAIK.
//     */
//    std::set<Index> find_separator(Index key) const;

//    /**
//     * Peforms a supposedly-faster (fewer matrix copy) version of elimination
//     * CURRENTLY IN TESTING
//     */
//    GaussianConditional::shared_ptr eliminateOneMatrixJoin(Index key);
//
//
//    /**
//     * Eliminate multiple variables at once, mostly used to eliminate frontal variables
//     */
//    GaussianBayesNet eliminateFrontals(const Ordering& frontals);

//    /**
//     * optimize a linear factor graph
//     * @param ordering fg in order
//     * @param enableJoinFactor uses the older joint factor combine process when true,
//     *    and when false uses the newer single matrix combine
//     */
//    VectorValues optimize(const Ordering& ordering, bool enableJoinFactor = true);

//    /**
//     * optimize a linear factor graph using a multi-frontal solver
//     * @param ordering fg in order
//     */
//    VectorValues optimizeMultiFrontals(const Ordering& ordering);

//    /**
//     * shared pointer versions for MATLAB
//     */
//    boost::shared_ptr<GaussianBayesNet> eliminate_(const Ordering&);
//    boost::shared_ptr<VectorValues> optimize_(const Ordering&);

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

//    /**
//     * Find all variables and their dimensions
//     * @return The set of all variable/dimension pairs
//     */
//    Dimensions dimensions() const;

    /**
     * Add zero-mean i.i.d. Gaussian prior terms to each variable
     * @param sigma Standard deviation of Gaussian
     */
    GaussianFactorGraph add_priors(double sigma, const GaussianVariableIndex<>& variableIndex) const;

//    /**
//     * Return RHS (b./sigmas) as Errors class
//     */
//    Errors rhs() const;
//
//    /**
//     * Return RHS (b./sigmas) as Vector
//     */
//    Vector rhsVector() const;
//
//    /**
//     * Return (dense) matrix associated with factor graph
//     * @param ordering of variables needed for matrix column order
//     */
//    std::pair<Matrix,Vector> matrix (const Ordering& ordering) const;

//    /**
//     * split the source vector w.r.t. the given ordering and assemble a vector config
//     * @param v: the source vector
//     * @param ordeirng: the ordering corresponding to the vector
//     */
//    VectorValues assembleValues(const Vector& v, const Ordering& ordering) const;
//
//    /**
//     * get the 1-based starting column indices for all variables
//     * @param ordering of variables needed for matrix column order
//     * @return The set of all variable/index pairs
//     */
//    std::pair<Dimensions, size_t> columnIndices_(const Ordering& ordering) const;
//    Dimensions columnIndices(const Ordering& ordering) const;
//
//    /**
//     * return the size of corresponding A matrix
//     */
//    std::pair<std::size_t, std::size_t> sizeOfA() const;

//  	/**
//  	 * Return 3*nzmax matrix where the rows correspond to the vectors i, j, and s
//  	 * to generate an m-by-n sparse matrix, which can be given to MATLAB's sparse function.
//  	 * The standard deviations are baked into A and b
//  	 * @param ordering of variables needed for matrix column order
//  	 */
//  	Matrix sparse(const Ordering& ordering) const;
//
//  	/**
//  	 * Version that takes column indices rather than ordering
//  	 */
//  	Matrix sparse(const Dimensions& indices) const;

//  	/**
//		 * Find solution using gradient descent
//		 * @param x0: VectorValues specifying initial estimate
//		 * @return solution
//		 */
//		VectorValues steepestDescent(const VectorValues& x0, bool verbose = false,
//				double epsilon = 1e-3, size_t maxIterations = 0) const;
//
//		/**
//		 * shared pointer versions for MATLAB
//		 */
//		boost::shared_ptr<VectorValues>
//		steepestDescent_(const VectorValues& x0, bool verbose = false,
//				double epsilon = 1e-3, size_t maxIterations = 0) const;
//
//		/**
//		 * Find solution using conjugate gradient descent
//		 * @param x0: VectorValues specifying initial estimate
//		 * @return solution
//		 */
//		VectorValues conjugateGradientDescent(const VectorValues& x0, bool verbose =
//				false, double epsilon = 1e-3, size_t maxIterations = 0) const;
//
//		/**
//		 * shared pointer versions for MATLAB
//		 */
//		boost::shared_ptr<VectorValues> conjugateGradientDescent_(
//				const VectorValues& x0, bool verbose = false, double epsilon = 1e-3,
//				size_t maxIterations = 0) const;
  };


  /* ************************************************************************* */
  template<class VariableIndexStorage>
  class GaussianVariableIndex : public VariableIndex<VariableIndexStorage> {
  public:
    typedef VariableIndex<VariableIndexStorage> Base;
    typedef typename VariableIndexStorage::template type_factory<size_t>::type storage_type;

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
    GaussianVariableIndex(const VariableIndex<VariableIndexStorage>& variableIndex, const GaussianFactorGraph& factorGraph);

    /**
     * Another constructor to upgrade from the base class using an existing
     * array of variable dimensions.
     */
    GaussianVariableIndex(const VariableIndex<VariableIndexStorage>& variableIndex, const storage_type& dimensions);

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
  template<class Storage>
  GaussianVariableIndex<Storage>::GaussianVariableIndex(const GaussianFactorGraph& factorGraph) :
  Base(factorGraph), dims_(Base::index_.size()) {
    fillDims(factorGraph); }

  /* ************************************************************************* */
  template<class Storage>
  GaussianVariableIndex<Storage>::GaussianVariableIndex(
      const VariableIndex<Storage>& variableIndex, const GaussianFactorGraph& factorGraph) :
      Base(variableIndex), dims_(Base::index_.size()) {
    fillDims(factorGraph); }

  /* ************************************************************************* */
  template<class Storage>
  GaussianVariableIndex<Storage>::GaussianVariableIndex(
      const VariableIndex<Storage>& variableIndex, const storage_type& dimensions) :
      Base(variableIndex), dims_(dimensions) {
    assert(Base::index_.size() == dims_.size()); }

  /* ************************************************************************* */
  template<class Storage>
  void GaussianVariableIndex<Storage>::fillDims(const GaussianFactorGraph& factorGraph) {
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
  template<class Storage>
  void GaussianVariableIndex<Storage>::permute(const Permutation& permutation) {
    VariableIndex<Storage>::permute(permutation);
    storage_type original(this->dims_.size());
    this->dims_.swap(original);
    for(Index j=0; j<permutation.size(); ++j)
      this->dims_[j] = original[permutation[j]];
  }

  /* ************************************************************************* */
  template<class Storage>
  void GaussianVariableIndex<Storage>::augment(const GaussianFactorGraph& factorGraph) {
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
//	template <class Factors>
//	std::pair<Matrix, SharedDiagonal> combineFactorsAndCreateMatrix(
//			const Factors& factors,
//			const Ordering& order, const Dimensions& dimensions);

} // namespace gtsam
