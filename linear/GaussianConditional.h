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

#include <list>
#include <boost/utility.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/numeric/ublas/triangular.hpp>

#include <gtsam/base/types.h>
#include <gtsam/inference/IndexConditional.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/blockMatrices.h>

namespace gtsam {

class GaussianFactor;

/**
 * A conditional Gaussian functions as the node in a Bayes network
 * It has a set of parents y,z, etc. and implements a probability density on x.
 * The negative log-probability is given by || Rx - (d - Sy - Tz - ...)||^2
 */
class GaussianConditional : public IndexConditional {

public:
  typedef GaussianFactor Factor;
	typedef boost::shared_ptr<GaussianConditional> shared_ptr;

	/** Store the conditional matrix as upper-triangular column-major */
	typedef boost::numeric::ublas::triangular_matrix<double, boost::numeric::ublas::upper, boost::numeric::ublas::column_major> matrix_type;
	typedef VerticalBlockView<matrix_type> rsd_type;

	typedef rsd_type::block_type r_type;
  typedef rsd_type::const_block_type const_r_type;
  typedef rsd_type::column_type d_type;
  typedef rsd_type::const_column_type const_d_type;

protected:

	/** Store the conditional as one big upper-triangular wide matrix, arranged
	 * as [ R S1 S2 ... d ].  Access these blocks using a VerticalBlockView. */
	matrix_type matrix_;
	rsd_type rsd_;

	/** vector of standard deviations */
	Vector sigmas_;

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
	 * constructor with number of arbitrary parents
	 * |Rx+sum(Ai*xi)-d|
	 */
	GaussianConditional(Index key, const Vector& d,
			const Matrix& R, const std::list<std::pair<Index, Matrix> >& parents, const Vector& sigmas);

	/**
	 * Constructor when matrices are already stored in a combined matrix, allows
	 * for multiple frontal variables.
	 */
	template<typename Iterator, class Matrix>
	GaussianConditional(Iterator firstKey, Iterator lastKey, size_t nrFrontals, const VerticalBlockView<Matrix>& matrices, const Vector& sigmas);

	/** print */
	void print(const std::string& = "GaussianConditional") const;

	/** equals function */
	bool equals(const GaussianConditional &cg, double tol = 1e-9) const;

//	/** permute the variables in the conditional */
//	void permuteWithInverse(const Permutation& inversePermutation);
//
//  /** Permute the variables when only separator variables need to be permuted.
//   * Returns true if any reordered variables appeared in the separator and
//   * false if not.
//   */
//  bool permuteSeparatorWithInverse(const Permutation& inversePermutation);

	/** dimension of multivariate variable */
	size_t dim() const { return rsd_.size1(); }

	/** return stuff contained in GaussianConditional */
	rsd_type::const_column_type get_d() const { return rsd_.column(1+nrParents(), 0); }
	rsd_type::const_block_type get_R() const { return rsd_(0); }
	rsd_type::const_block_type get_S(const_iterator variable) const { return rsd_(variable - this->begin()); }
	const Vector& get_sigmas() const {return sigmas_;}

	/**
	 * solve a conditional Gaussian
	 * @param x values structure in which the parents values (y,z,...) are known
	 * @return solution x = R \ (d - Sy - Tz - ...)
	 */
	Vector solve(const VectorValues& x) const;

  /**
   * solve a conditional Gaussian
   * @param x values structure in which the parents values (y,z,...) are known
   * @return solution x = R \ (d - Sy - Tz - ...)
   */
  Vector solve(const Permuted<VectorValues>& x) const;

protected:
  rsd_type::column_type get_d_() { return rsd_.column(1+nrParents(), 0); }
  rsd_type::block_type get_R_() { return rsd_(0); }
  rsd_type::block_type get_S_(iterator variable) { return rsd_(variable - this->begin()); }

  friend class GaussianFactor;

private:
//	/** Serialization function */
//	friend class boost::serialization::access;
//	template<class Archive>
//	void serialize(Archive & ar, const unsigned int version) {
//		ar & boost::serialization::make_nvp("Conditional", boost::serialization::base_object<Conditional>(*this));
//		ar & BOOST_SERIALIZATION_NVP(R_);
//		ar & BOOST_SERIALIZATION_NVP(parents_);
//		ar & BOOST_SERIALIZATION_NVP(d_);
//		ar & BOOST_SERIALIZATION_NVP(sigmas_);
//	}
};

/* ************************************************************************* */
	template<typename Iterator, class Matrix>
	GaussianConditional::GaussianConditional(Iterator firstKey, Iterator lastKey,
			size_t nrFrontals, const VerticalBlockView<Matrix>& matrices,
			const Vector& sigmas) :
		rsd_(matrix_), sigmas_(sigmas) {
  	nrFrontals_ = nrFrontals;
		std::copy(firstKey, lastKey, back_inserter(keys_));
		rsd_.assignNoalias(matrices);
	}

	/* ************************************************************************* */

}
