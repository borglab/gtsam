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
#include <gtsam/base/blockMatrices.h>
#include <gtsam/inference/IndexConditional.h>
#include <gtsam/linear/VectorValues.h>

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
class JacobianFactor;

/**
 * A conditional Gaussian functions as the node in a Bayes network
 * It has a set of parents y,z, etc. and implements a probability density on x.
 * The negative log-probability is given by \f$ \frac{1}{2} |Rx - (d - Sy - Tz - ...)|^2 \f$
 */
class GaussianConditional : public IndexConditional {

public:
  typedef GaussianFactor FactorType;
	typedef boost::shared_ptr<GaussianConditional> shared_ptr;

	/** Store the conditional matrix as upper-triangular column-major */
	typedef Matrix RdMatrix;
	typedef VerticalBlockView<RdMatrix> rsd_type;

	typedef rsd_type::Block r_type;
  typedef rsd_type::constBlock const_r_type;
  typedef rsd_type::Column d_type;
  typedef rsd_type::constColumn const_d_type;

protected:

	/** Store the conditional as one big upper-triangular wide matrix, arranged
	 * as \f$ [ R S1 S2 ... d ] \f$.  Access these blocks using a VerticalBlockView.
	 * */
	RdMatrix matrix_;
	rsd_type rsd_;

	/** vector of standard deviations */
	Vector sigmas_;

  /** typedef to base class */
  typedef IndexConditional Base;

public:

	/** default constructor needed for serialization */
	GaussianConditional();

	/** constructor */
	explicit GaussianConditional(Index key);

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
	 * \f$ |Rx+sum(Ai*xi)-d| \f$
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
      const VerticalBlockView<MATRIX>& matrices, const Vector& sigmas);

  /** Copy constructor */
	GaussianConditional(const GaussianConditional& rhs);

	/** Combine several GaussianConditional into a single dense GC.  The
	 * conditionals enumerated by \c first and \c last must be in increasing
	 * order, meaning that the parents of any conditional may not include a
	 * conditional coming before it.
	 * @param firstConditional Iterator to the first conditional to combine, must dereference to a shared_ptr<GaussianConditional>.
   * @param lastConditional Iterator to after the last conditional to combine, must dereference to a shared_ptr<GaussianConditional>. */
	template<typename ITERATOR>
	static shared_ptr Combine(ITERATOR firstConditional, ITERATOR lastConditional);

	/** Assignment operator */
	GaussianConditional& operator=(const GaussianConditional& rhs);

	/** print */
	void print(const std::string& = "GaussianConditional",
			const IndexFormatter& formatter = DefaultIndexFormatter) const;

	/** equals function */
	bool equals(const GaussianConditional &cg, double tol = 1e-9) const;

	/** dimension of multivariate variable */
	size_t dim() const { return rsd_.rows(); }

	/** Compute the information matrix */
	Matrix computeInformation() const {
	  return get_R().transpose() * get_R();
	}

	/** Return a view of the upper-triangular R block of the conditional */
	rsd_type::constBlock get_R() const { return rsd_.range(0, nrFrontals()); }

	/** Return a view of the r.h.s. d vector */
	const_d_type get_d() const { return rsd_.column(nrFrontals()+nrParents(), 0); }

	/** get the dimension of a variable */
	size_t dim(const_iterator variable) const { return rsd_(variable - this->begin()).cols(); }

	/** Get a view of the parent block corresponding to the variable pointed to by the given key iterator */
	rsd_type::constBlock get_S(const_iterator variable) const { return rsd_(variable - this->begin()); }
  /** Get a view of the parent block corresponding to the variable pointed to by the given key iterator (non-const version) */
	rsd_type::constBlock get_S() const { return rsd_.range(nrFrontals(), size()); }
	/** Get the Vector of sigmas */
	const Vector& get_sigmas() const {return sigmas_;}

protected:

	const RdMatrix& matrix() const { return matrix_; }
	const rsd_type& rsd() const { return rsd_; }

public:
	/**
	 * Copy to a Factor (this creates a JacobianFactor and returns it as its
	 * base class GaussianFactor.
	 */
	boost::shared_ptr<JacobianFactor> toFactor() const;

  /**
   * Solves a conditional Gaussian and writes the solution into the entries of
   * \c x for each frontal variable of the conditional.  The parents are
   * assumed to have already been solved in and their values are read from \c x.
   * This function works for multiple frontal variables.
   *
   * Given the Gaussian conditional with log likelihood \f$ |R x_f - (d - S x_s)|^2,
   * where \f$ f \f$ are the frontal variables and \f$ s \f$ are the separator
   * variables of this conditional, this solve function computes
   * \f$ x_f = R^{-1} (d - S x_s) \f$ using back-substitution.
   *
   * @param x VectorValues structure with solved parents \f$ x_s \f$, and into which the
   * solution \f$ x_f \f$ will be written.
   */
  void solveInPlace(VectorValues& x) const;

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
		const Vector& sigmas) :
	IndexConditional(std::vector<Index>(firstKey, lastKey), nrFrontals), rsd_(
			matrix_), sigmas_(sigmas) {
	rsd_.assignNoalias(matrices);
}

/* ************************************************************************* */
template<typename ITERATOR>
GaussianConditional::shared_ptr GaussianConditional::Combine(ITERATOR firstConditional, ITERATOR lastConditional) {

  // TODO:  check for being a clique

  // Get dimensions from first conditional
  std::vector<size_t> dims;   dims.reserve((*firstConditional)->size() + 1);
  for(const_iterator j = (*firstConditional)->begin(); j != (*firstConditional)->end(); ++j)
    dims.push_back((*firstConditional)->dim(j));
  dims.push_back(1);

  // We assume the conditionals form clique, so the first n variables will be
  // frontal variables in the new conditional.
  size_t nFrontals = 0;
  size_t nRows = 0;
  for(ITERATOR c = firstConditional; c != lastConditional; ++c) {
    nRows += dims[nFrontals];
    ++ nFrontals;
  }

  // Allocate combined conditional, has same keys as firstConditional
  Matrix tempCombined;
  VerticalBlockView<Matrix> tempBlockView(tempCombined, dims.begin(), dims.end(), 0);
  GaussianConditional::shared_ptr combinedConditional(new GaussianConditional((*firstConditional)->begin(), (*firstConditional)->end(), nFrontals, tempBlockView, zero(nRows)));

  // Resize to correct number of rows
  combinedConditional->matrix_.resize(nRows, combinedConditional->matrix_.cols());
  combinedConditional->rsd_.rowEnd() = combinedConditional->matrix_.rows();

  // Copy matrix and sigmas
  const size_t totalDims = combinedConditional->matrix_.cols();
  size_t currentSlot = 0;
  for(ITERATOR c = firstConditional; c != lastConditional; ++c) {
    const size_t startRow = combinedConditional->rsd_.offset(currentSlot); // Start row is same as start column
    combinedConditional->rsd_.range(0, currentSlot).block(startRow, 0, dims[currentSlot], combinedConditional->rsd_.offset(currentSlot)).operator=(
        Matrix::Zero(dims[currentSlot], combinedConditional->rsd_.offset(currentSlot)));
    combinedConditional->rsd_.range(currentSlot, dims.size()).block(startRow, 0, dims[currentSlot], totalDims - startRow).operator=(
        (*c)->matrix_);
    combinedConditional->sigmas_.segment(startRow, dims[currentSlot]) = (*c)->sigmas_;
    ++ currentSlot;
  }

  return combinedConditional;
}

} // gtsam
