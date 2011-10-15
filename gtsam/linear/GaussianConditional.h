/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussianConditional.h
 * @brief   Conditional Gaussian Base class
 * @author  Christian Potthast
 */

// \callgraph

#pragma once

#include <boost/utility.hpp>

#include <gtsam/base/types.h>
#include <gtsam/inference/IndexConditional.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/base/blockMatrices.h>

// Forward declaration to friend unit tests
class eliminate2JacobianFactorTest;
class constructorGaussianConditionalTest;
class eliminationGaussianFactorGraphTest;
class complicatedMarginalGaussianJunctionTreeTest;
class computeInformationGaussianConditionalTest;
class isGaussianFactorGaussianConditionalTest;

namespace gtsam {

// Forward declarations
class GaussianFactor;

/**
 * A conditional Gaussian functions as the node in a Bayes network
 * It has a set of parents y,z, etc. and implements a probability density on x.
 * The negative log-probability is given by || Rx - (d - Sy - Tz - ...)||^2
 */
class GaussianConditional : public IndexConditional {

public:
  typedef GaussianFactor FactorType;
	typedef boost::shared_ptr<GaussianConditional> shared_ptr;

	/** Store the conditional matrix as upper-triangular column-major */
	typedef Matrix AbMatrix;
	typedef VerticalBlockView<AbMatrix> rsd_type;

	typedef rsd_type::Block r_type;
  typedef rsd_type::constBlock const_r_type;
  typedef rsd_type::Column d_type;
  typedef rsd_type::constColumn const_d_type;

  typedef Eigen::LDLT<Matrix>::TranspositionType TranspositionType;

protected:

	/** Store the conditional as one big upper-triangular wide matrix, arranged
	 * as [ R S1 S2 ... d ].  Access these blocks using a VerticalBlockView.
	 *
	 * WARNING!!! When using with LDL, R is the permuted upper triangular matrix.
	 * Its columns/rows do not correspond to the correct components of the variables.
	 * Use R*permutation_ to get back the correct non-permuted order,
	 * for example when converting to the Jacobian
	 * */
	AbMatrix matrix_;
	rsd_type rsd_;

	/** vector of standard deviations */
	Vector sigmas_;

  /** Store the permutation matrix, used by LDL' in the pivoting process
   * This is used to get back the correct ordering of x after solving by backSubstitition */
  TranspositionType permutation_;

public:

	/** default constructor needed for serialization */
	GaussianConditional();

	/** constructor */
	GaussianConditional(Index key);

	/** constructor with no parents
	 * |Rx-d|
	 */
	GaussianConditional(Index key, const Vector& d, const Matrix& R, const Vector& sigmas);

	/** constructor with only one parent
	 * |Rx+Sy-d|
	 */
	GaussianConditional(Index key, const Vector& d, const Matrix& R,
			Index name1, const Matrix& S, const Vector& sigmas);

	/** constructor with two parents
	 * |Rx+Sy+Tz-d|
	 */
	GaussianConditional(Index key, const Vector& d, const Matrix& R,
			Index name1, const Matrix& S, Index name2, const Matrix& T, const Vector& sigmas);

	/**
	 * constructor with number of arbitrary parents (only used in unit tests,
	 * std::list is not efficient)
	 * |Rx+sum(Ai*xi)-d|
	 */
	GaussianConditional(Index key, const Vector& d,
			const Matrix& R, const std::list<std::pair<Index, Matrix> >& parents, const Vector& sigmas);

	/**
	 * Constructor with arbitrary number of frontals and parents (only used in unit tests,
   * std::list is not efficient)
	 */
	GaussianConditional(const std::list<std::pair<Index, Matrix> >& terms,
	    size_t nrFrontals, const Vector& d, const Vector& sigmas);

	/**
	 * Constructor when matrices are already stored in a combined matrix, allows
	 * for multiple frontal variables.
	 */
	template<typename ITERATOR, class MATRIX>
	GaussianConditional(ITERATOR firstKey, ITERATOR lastKey, size_t nrFrontals,
      const VerticalBlockView<MATRIX>& matrices, const Vector& sigmas,
      const TranspositionType& permutation = TranspositionType());

	/** print */
	void print(const std::string& = "GaussianConditional") const;

	/** equals function */
	bool equals(const GaussianConditional &cg, double tol = 1e-9) const;

	/** dimension of multivariate variable */
	size_t dim() const { return rsd_.rows(); }

	/** Compute the information matrix */
	Matrix computeInformation() const {
	  Matrix R = get_R() * permutation_.transpose();
	  return R.transpose() * R;
	}

	/** Return a view of the upper-triangular R block of the conditional */
	rsd_type::constBlock get_R() const { return rsd_.range(0, nrFrontals()); }

	/** Return a view of the r.h.s. d vector */
	const_d_type get_d() const { return rsd_.column(nrFrontals()+nrParents(), 0); }

	/**
	 * get a RHS as a single vector
	 * FIXME: shouldn't this be weighted by sigmas?
	 */
	Vector rhs() const { return get_d(); }

	/** get the dimension of a variable */
	size_t dim(const_iterator variable) const { return rsd_(variable - this->begin()).cols(); }

	/** Get a view of the parent block corresponding to the variable pointed to by the given key iterator */
	rsd_type::constBlock get_S(const_iterator variable) const { return rsd_(variable - this->begin()); }
  /** Get a view of the parent block corresponding to the variable pointed to by the given key iterator (non-const version) */
	rsd_type::constBlock get_S() const { return rsd_.range(nrFrontals(), size()); }
	/** Get the Vector of sigmas */
	const Vector& get_sigmas() const {return sigmas_;}

	/** Return the permutation of R made by LDL, to recover R in the correct
	 * order use R*permutation, see the GaussianConditional main class comment
	 */
	const TranspositionType& permutation() const { return permutation_; }

protected:

	const AbMatrix& matrix() const { return matrix_; }
	const rsd_type& rsd() const { return rsd_; }

public:
	/**
	 * Copy to a Factor (this creates a JacobianFactor and returns it as its
	 * base class GaussianFactor.
	 */
	boost::shared_ptr<JacobianFactor> toFactor() const;

	/**
	 * Adds the RHS to a given VectorValues for use in solve() functions.
	 * @param x is the values to be updated, assumed allocated
	 */
	void rhs(VectorValues& x) const;

  /**
   * Adds the RHS to a given VectorValues for use in solve() functions.
   * @param x is the values to be updated, assumed allocated
   */
  void rhs(Permuted<VectorValues>& x) const;

  /**
   * solves a conditional Gaussian and stores the result in x
   * This function works for multiple frontal variables.
   * NOTE: assumes that the RHS for the frontals is stored in x, and
   * then replaces the RHS with the partial result for this conditional,
   * assuming that parents have been solved already.
   *
   * @param x values structure with solved parents, and the RHS for this conditional
   * @return solution x = R \ (d - Sy - Tz - ...) for each frontal variable
   */
  void solveInPlace(VectorValues& x) const;

  /**
   * solves a conditional Gaussian and stores the result in x
   * Identical to solveInPlace() above, with a permuted x
   */
  void solveInPlace(Permuted<VectorValues>& x) const;

  /**
   * Solves a conditional Gaussian and returns a new VectorValues
   * This function works for multiple frontal variables, but should
   * only be used for testing as it copies the input vector values
   *
   * Assumes, as in solveInPlace, that the RHS has been stored in x
   * for all frontal variables
   */
  VectorValues solve(const VectorValues& x) const;

  // functions for transpose backsubstitution

  /**
   * Performs backsubstition in place on values
   */
	void solveTransposeInPlace(VectorValues& gy) const;
	void scaleFrontalsBySigma(VectorValues& gy) const;

protected:
  rsd_type::Column get_d_() { return rsd_.column(nrFrontals()+nrParents(), 0); }
  rsd_type::Block get_R_() { return rsd_.range(0, nrFrontals()); }
  rsd_type::Block get_S_(iterator variable) { return rsd_(variable - this->begin()); }

private:

  // Friends
  friend class JacobianFactor;
  friend class ::eliminate2JacobianFactorTest;
  friend class ::constructorGaussianConditionalTest;
  friend class ::eliminationGaussianFactorGraphTest;
  friend class ::complicatedMarginalGaussianJunctionTreeTest;
  friend class ::computeInformationGaussianConditionalTest;
  friend class ::isGaussianFactorGaussianConditionalTest;

  /** Serialization function */
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version) {
		ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(IndexConditional);
		ar & BOOST_SERIALIZATION_NVP(matrix_);
		ar & BOOST_SERIALIZATION_NVP(rsd_);
		ar & BOOST_SERIALIZATION_NVP(sigmas_);
	}
}; // GaussianConditional

/* ************************************************************************* */
template<typename ITERATOR, class MATRIX>
GaussianConditional::GaussianConditional(ITERATOR firstKey, ITERATOR lastKey,
		size_t nrFrontals, const VerticalBlockView<MATRIX>& matrices,
		const Vector& sigmas, const GaussianConditional::TranspositionType& permutation) :
	IndexConditional(std::vector<Index>(firstKey, lastKey), nrFrontals), rsd_(
			matrix_), sigmas_(sigmas), permutation_(permutation) {
	rsd_.assignNoalias(matrices);
}

/* ************************************************************************* */

} // gtsam
