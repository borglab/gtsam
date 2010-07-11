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
#include <boost/serialization/map.hpp>
#include <boost/foreach.hpp>
#include <list>
#include <set>

#include "Factor.h"
#include "Matrix.h"
#include "VectorConfig.h"
#include "SharedDiagonal.h"
#include "GaussianConditional.h" // Needed for MATLAB
#include "GaussianBayesNet.h"
#include "SymbolMap.h"

namespace gtsam {

	class Ordering;

/**
 * Base Class for a linear factor.
 * GaussianFactor is non-mutable (all methods const!).
 * The factor value is exp(-0.5*||Ax-b||^2)
 */
class GaussianFactor: boost::noncopyable, public Factor<VectorConfig> {
public:

	typedef boost::shared_ptr<GaussianFactor> shared_ptr;
	typedef SymbolMap<Matrix>::iterator iterator;
	typedef SymbolMap<Matrix>::const_iterator const_iterator;

protected:

	SharedDiagonal model_; // Gaussian noise model with diagonal covariance matrix
	SymbolMap<Matrix> As_; // linear matrices
	Vector b_; // right-hand-side

public:

	/* default constructor for I/O */
	GaussianFactor()  {}

	/** Construct Null factor */
	GaussianFactor(const Vector& b_in);

	/** Construct unary factor */
	GaussianFactor(const Symbol& key1, const Matrix& A1,
			const Vector& b, const SharedDiagonal& model);

	/** Construct binary factor */
	GaussianFactor(const Symbol& key1, const Matrix& A1,
			const Symbol& key2, const Matrix& A2,
			const Vector& b, const SharedDiagonal& model);

	/** Construct ternary factor */
	GaussianFactor(const Symbol& key1, const Matrix& A1, const Symbol& key2,
			const Matrix& A2, const Symbol& key3, const Matrix& A3,
			const Vector& b, const SharedDiagonal& model);

	/** Construct an n-ary factor */
	GaussianFactor(const std::vector<std::pair<Symbol, Matrix> > &terms,
	    const Vector &b, const SharedDiagonal& model);

	GaussianFactor(const std::list<std::pair<Symbol, Matrix> > &terms,
	    const Vector &b, const SharedDiagonal& model);

	/** Construct from Conditional Gaussian */
	GaussianFactor(const boost::shared_ptr<GaussianConditional>& cg);

	/** Constructor that combines a set of factors */
	GaussianFactor(const std::vector<shared_ptr> & factors);

	// Implementing Testable virtual functions

	void print(const std::string& s = "") const;
	bool equals(const Factor<VectorConfig>& lf, double tol = 1e-9) const;

	// Implementing Factor virtual functions

	Vector unweighted_error(const VectorConfig& c) const; /** (A*x-b) */
	Vector error_vector(const VectorConfig& c) const; /** (A*x-b)/sigma */
	double error(const VectorConfig& c) const; /**  0.5*(A*x-b)'*D*(A*x-b) */
	std::size_t size() const { return As_.size();}

	/** STL like, return the iterator pointing to the first node */
	const_iterator const begin() const { return As_.begin();}

	/** STL like, return the iterator pointing to the last node */
	const_iterator const end() const { return As_.end();	}

	/** check if empty */
	bool empty() const { return b_.size() == 0;}

	/** get a copy of b */
	const Vector& get_b() const {	return b_;	}

	/** get a copy of sigmas */
	const Vector& get_sigmas() const {	return model_->sigmas();	}

	/** get a copy of model */
	const SharedDiagonal& get_model() const { return model_;  }

	/**
	 * get a copy of the A matrix from a specific node
	 * O(log n)
	 */
	const Matrix& get_A(const Symbol& key) const {
		return As_.at(key);
	}

	/** erase the A associated with the input key */
	size_t erase_A(const Symbol& key) { return As_.erase(key); }

	/** operator[] syntax for get */
	inline const Matrix& operator[](const Symbol& name) const {
		return get_A(name);
	}

	/** Check if factor involves variable with key */
	bool involves(const Symbol& key) const {
		const_iterator it = As_.find(key);
		return (it != As_.end());
	}

	/**
	 * return the number of rows from the b vector
	 * @return a integer with the number of rows from the b vector
	 */
	int numberOfRows() const { return b_.size();}

	/**
	 * Find all variables
	 * @return The set of all variable keys
	 */
	std::list<Symbol> keys() const;

	/**
	 * return the first key
	 * TODO: this function should be removed and the minimum spanning tree code
	 * modified accordingly.
	 * @return The set of all variable keys
	 */
	Symbol key1() const { return As_.begin()->first; }

	/**
	 * return the first key
   * TODO: this function should be removed and the minimum spanning tree code
   * modified accordingly.
	 * @return The set of all variable keys
	 */
	Symbol key2() const {
		if (As_.size() < 2) throw std::invalid_argument("GaussianFactor: less than 2 keys!");
		return (++(As_.begin()))->first;
	}


	/**
	 * Find all variables and their dimensions
	 * @return The set of all variable/dimension pairs
	 */
	Dimensions dimensions() const;

	/**
	 * Get the dimension of a particular variable
	 * @param key is the name of the variable
	 * @return the size of the variable
	 */
	size_t getDim(const Symbol& key) const;

	/**
	 * Add to separator set if this factor involves key, but don't add key itself
	 * @param key
	 * @param separator set to add to
	 */
	void tally_separator(const Symbol& key,
			std::set<Symbol>& separator) const;

	/** Return A*x */
	Vector operator*(const VectorConfig& x) const;

	/** Return A'*e */
	VectorConfig operator^(const Vector& e) const;

	/** x += A'*e */
	void transposeMultiplyAdd(double alpha, const Vector& e, VectorConfig& x) const;

	/**
	 * Return (dense) matrix associated with factor
	 * @param ordering of variables needed for matrix column order
	 * @param set weight to true to bake in the weights
	 */
	std::pair<Matrix, Vector> matrix(const Ordering& ordering, bool weight = true) const;

	/**
	 * Return (dense) matrix associated with factor
	 * The returned system is an augmented matrix: [A b]
	 * @param ordering of variables needed for matrix column order
	 * @param set weight to use whitening to bake in weights
	 */
	Matrix matrix_augmented(const Ordering& ordering, bool weight = true) const;

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

	/** insert, copies A */
	void insert(const Symbol& key, const Matrix& A) {
		As_.insert(std::make_pair(key, A));
	}

	/** set RHS, copies b */
	void set_b(const Vector& b) {
		this->b_ = b;
	}

	// set A matrices for the linear factor, same as insert ?
	inline void set_A(const Symbol& key, const Matrix &A) {
		insert(key, A);
	}

	/**
	 * Current Implementation: Full QR factorization
	 * eliminate (in place!) one of the variables connected to this factor
	 * @param key the key of the node to be eliminated
	 * @return a new factor and a conditional gaussian on the eliminated variable
	 */
	std::pair<boost::shared_ptr<GaussianConditional>, shared_ptr>
	eliminate(const Symbol& key) const;

	/**
	 * Performs elimination given an augmented matrix
	 * @param
	 */
	static std::pair<GaussianBayesNet, shared_ptr>
	eliminateMatrix(Matrix& Ab, SharedDiagonal model,
			const Ordering& frontal, const Ordering& separator,
			const Dimensions& dimensions);

	static std::pair<GaussianConditional::shared_ptr, shared_ptr>
	eliminateMatrix(Matrix& Ab, SharedDiagonal model,
			const Symbol& frontal, const Ordering& separator,
			const Dimensions& dimensions);


	/**
	 * Take the factor f, and append to current matrices. Not very general.
	 * @param f linear factor graph
	 * @param m final number of rows of f, needs to be known in advance
	 * @param pos where to insert in the m-sized matrices
	 */
	void append_factor(GaussianFactor::shared_ptr f, size_t m, size_t pos);

}; // GaussianFactor

/* ************************************************************************* */

/**
 * creates a C++ string a la "x3", "m768"
 * @param c the base character
 * @param index the integer to be added
 * @return a C++ string
 */
std::string symbol(char c, int index);

} // namespace gtsam
