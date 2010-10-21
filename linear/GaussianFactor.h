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
 * @author  Christian Potthast
 */

// \callgraph

#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/tuple/tuple.hpp>
//#include <boost/serialization/map.hpp>
#include <boost/foreach.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/bind.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/pool/pool_alloc.hpp>
#include <list>
#include <set>
#include <vector>
#include <map>
#include <deque>

#include <gtsam/base/types.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/blockMatrices.h>
#include <gtsam/inference/IndexFactor.h>
#include <gtsam/inference/inference.h>
#include <gtsam/inference/VariableSlots.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/SharedDiagonal.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/GaussianBayesNet.h>

namespace gtsam {

class GaussianFactorGraph;
template<class VARIABLEINDEXSTORAGE=VariableIndexStorage_vector> class GaussianVariableIndex;

/** A map from key to dimension, useful in various contexts */
typedef std::map<Index, size_t> Dimensions;

/**
 * Base Class for a linear factor.
 * GaussianFactor is non-mutable (all methods const!).
 * The factor value is exp(-0.5*||Ax-b||^2)
 */
class GaussianFactor: public IndexFactor {

public:

  typedef GaussianConditional Conditional;
	typedef boost::shared_ptr<GaussianFactor> shared_ptr;
  typedef boost::numeric::ublas::matrix<double, boost::numeric::ublas::column_major> matrix_type;
	typedef VerticalBlockView<matrix_type> ab_type;

protected:

	SharedDiagonal model_; // Gaussian noise model with diagonal covariance matrix
	std::vector<size_t> firstNonzeroBlocks_;
	matrix_type matrix_;
	ab_type Ab_;

public:

	/** Copy constructor */
	GaussianFactor(const GaussianFactor& gf);

	/** default constructor for I/O */
	GaussianFactor();

	/** Construct Null factor */
	GaussianFactor(const Vector& b_in);

	/** Construct unary factor */
	GaussianFactor(Index i1, const Matrix& A1,
			const Vector& b, const SharedDiagonal& model);

	/** Construct binary factor */
	GaussianFactor(Index i1, const Matrix& A1,
			Index i2, const Matrix& A2,
			const Vector& b, const SharedDiagonal& model);

	/** Construct ternary factor */
	GaussianFactor(Index i1, const Matrix& A1, Index i2,
			const Matrix& A2, Index i3, const Matrix& A3,
			const Vector& b, const SharedDiagonal& model);

	/** Construct an n-ary factor */
	GaussianFactor(const std::vector<std::pair<Index, Matrix> > &terms,
	    const Vector &b, const SharedDiagonal& model);

	GaussianFactor(const std::list<std::pair<Index, Matrix> > &terms,
	    const Vector &b, const SharedDiagonal& model);

	/** Construct from Conditional Gaussian */
	GaussianFactor(const GaussianConditional& cg);

//	/** Constructor that combines a set of factors */
//	GaussianFactor(const std::vector<shared_ptr> & factors);

	// Implementing Testable interface
	void print(const std::string& s = "") const;
	bool equals(const GaussianFactor& lf, double tol = 1e-9) const;

	Vector unweighted_error(const VectorValues& c) const; /** (A*x-b) */
	Vector error_vector(const VectorValues& c) const; /** (A*x-b)/sigma */
	double error(const VectorValues& c) const; /**  0.5*(A*x-b)'*D*(A*x-b) */

	/** Check if the factor contains no information, i.e. zero rows.  This does
	 * not necessarily mean that the factor involves no variables (to check for
	 * involving no variables use keys().empty()).
	 */
	bool empty() const { return Ab_.size1() == 0;}

	/** Get a view of the r.h.s. vector b */
	ab_type::const_column_type getb() const { return Ab_.column(size(), 0); }
  ab_type::column_type getb() { return Ab_.column(size(), 0); }

	/** Get a view of the A matrix for the variable pointed to be the given key iterator */
	ab_type::const_block_type getA(const_iterator variable) const { return Ab_(variable - keys_.begin());	}
  ab_type::block_type getA(iterator variable) { return Ab_(variable - keys_.begin()); }

	/** Return the dimension of the variable pointed to by the given key iterator
	 * todo: Remove this in favor of keeping track of dimensions with variables?
	 */
	size_t getDim(const_iterator variable) const { return Ab_(variable - keys_.begin()).size2(); }

  /**
   * Permutes the GaussianFactor, but for efficiency requires the permutation
   * to already be inverted.  This acts just as a change-of-name for each
   * variable.  The order of the variables within the factor is not changed.
   */
  void permuteWithInverse(const Permutation& inversePermutation);

  /**
   * Named constructor for combining a set of factors with pre-computed set of variables.
   */
  template<class STORAGE>
  static shared_ptr Combine(const FactorGraph<GaussianFactor>& factorGraph,
      const GaussianVariableIndex<STORAGE>& variableIndex, const std::vector<size_t>& factorIndices,
      const std::vector<Index>& variables, const std::vector<std::vector<size_t> >& variablePositions);

  /**
   * Named constructor for combining a set of factors with pre-computed set of variables.
   */
  static shared_ptr Combine(const FactorGraph<GaussianFactor>& factors, const VariableSlots& variableSlots);

protected:

  /** Protected mutable accessor for the r.h.s. b. */

  /** Internal debug check to make sure variables are sorted */
  void assertInvariants() const;

public:

	/** get a copy of sigmas */
	const Vector& get_sigmas() const {	return model_->sigmas();	}

	/** get a copy of model */
	const SharedDiagonal& get_model() const { return model_;  }

	/**
	 * return the number of rows from the b vector
	 * @return a integer with the number of rows from the b vector
	 */
	size_t numberOfRows() const { return Ab_.size1(); }

	/** Return A*x */
	Vector operator*(const VectorValues& x) const;

//	/** Return A'*e */
//	VectorValues operator^(const Vector& e) const;

	/** x += A'*e */
	void transposeMultiplyAdd(double alpha, const Vector& e, VectorValues& x) const;

	/**
	 * Return (dense) matrix associated with factor
	 * @param ordering of variables needed for matrix column order
	 * @param set weight to true to bake in the weights
	 */
	std::pair<Matrix, Vector> matrix(bool weight = true) const;

	/**
	 * Return (dense) matrix associated with factor
	 * The returned system is an augmented matrix: [A b]
	 * @param ordering of variables needed for matrix column order
	 * @param set weight to use whitening to bake in weights
	 */
	Matrix matrix_augmented(bool weight = true) const;

	/**
	 * Return vectors i, j, and s to generate an m-by-n sparse matrix
	 * such that S(i(k),j(k)) = s(k), which can be given to MATLAB's sparse.
	 * As above, the standard deviations are baked into A and b
	 * @param first column index for each variable
	 */
	boost::tuple<std::list<int>, std::list<int>, std::list<double> >
		sparse(const Dimensions& columnIndices) const;

	/* ************************************************************************* */
	// MUTABLE functions. FD:on the path to being eradicated
	/* ************************************************************************* */

	GaussianConditional::shared_ptr eliminateFirst();

  GaussianBayesNet::shared_ptr eliminate(size_t nrFrontals = 1);

	friend class GaussianFactorGraph;
	friend class Inference;

}; // GaussianFactor


} // namespace gtsam
