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

#include "Factor.h"
#include "Matrix.h"
#include "VectorConfig.h"

namespace gtsam {

	class GaussianConditional;
	class Ordering;

/**
 * Base Class for a linear factor.
 * GaussianFactor is non-mutable (all methods const!).
 * The factor value is exp(-0.5*||Ax-b||^2)
 */
class GaussianFactor: public Factor<VectorConfig> {
public:

	typedef boost::shared_ptr<GaussianFactor> shared_ptr;
	typedef std::map<std::string, Matrix>::iterator iterator;
	typedef std::map<std::string, Matrix>::const_iterator const_iterator;

protected:

	std::map<std::string, Matrix> As_; // linear matrices
	Vector b_; // right-hand-side
	Vector sigmas_; // vector of standard deviations for each row in the factor

public:

	// TODO: eradicate, as implies non-const
	GaussianFactor()  {
	}

	/** Construct Null factor */
	GaussianFactor(const Vector& b_in) :
		b_(b_in), sigmas_(ones(b_in.size())){
	}

	/** Construct unary factor */
	GaussianFactor(const std::string& key1, const Matrix& A1,
			const Vector& b, double sigma) :
		b_(b), sigmas_(repeat(b.size(),sigma)) {
		As_.insert(make_pair(key1, A1));
	}

	/** Construct unary factor with vector of sigmas*/
	GaussianFactor(const std::string& key1, const Matrix& A1,
			const Vector& b, const Vector& sigmas) :
		b_(b), sigmas_(sigmas) {
		As_.insert(make_pair(key1, A1));
	}

	/** Construct binary factor */
	GaussianFactor(const std::string& key1, const Matrix& A1,
			const std::string& key2, const Matrix& A2,
			const Vector& b, double sigma) :
		b_(b), sigmas_(repeat(b.size(),sigma))  {
		As_.insert(make_pair(key1, A1));
		As_.insert(make_pair(key2, A2));
	}

	/** Construct ternary factor */
	GaussianFactor(const std::string& key1, const Matrix& A1,
			const std::string& key2, const Matrix& A2,
			const std::string& key3, const Matrix& A3,
			const Vector& b, double sigma) :
		b_(b), sigmas_(repeat(b.size(),sigma))  {
		As_.insert(make_pair(key1, A1));
		As_.insert(make_pair(key2, A2));
		As_.insert(make_pair(key3, A3));
	}

	/** Construct an n-ary factor */
	GaussianFactor(const std::vector<std::pair<std::string, Matrix> > &terms,
	    const Vector &b, double sigma) :
	    b_(b), sigmas_(repeat(b.size(),sigma))  {
	  for(unsigned int i=0; i<terms.size(); i++)
	    As_.insert(terms[i]);
	}

	/** Construct an n-ary factor with a multiple sigmas*/
	GaussianFactor(const std::vector<std::pair<std::string, Matrix> > &terms,
				const Vector &b, const Vector& sigmas) :
			b_(b), sigmas_(sigmas) {
			for (unsigned int i = 0; i < terms.size(); i++)
				As_.insert(terms[i]);
		}

	/** Construct from Conditional Gaussian */
	GaussianFactor(const boost::shared_ptr<GaussianConditional>& cg);

	/**
	 * Constructor that combines a set of factors
	 * @param factors Set of factors to combine
	 */
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
	const Vector& get_sigmas() const {	return sigmas_;	}

	/**
	 * get a copy of the A matrix from a specific node
	 * O(log n)
	 */
	const Matrix& get_A(const std::string& key) const {
		const_iterator it = As_.find(key);
		if (it == As_.end())
			throw(std::invalid_argument("GaussianFactor::[] invalid key: " + key));
		return it->second;
	}

	/** operator[] syntax for get */
	inline const Matrix& operator[](const std::string& name) const {
		return get_A(name);
	}

	/** Check if factor involves variable with key */
	bool involves(const std::string& key) const {
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
	std::list<std::string> keys() const;

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
	size_t getDim(const std::string& key) const;

	/**
	 * Add to separator set if this factor involves key, but don't add key itself
	 * @param key
	 * @param separator set to add to
	 */
	void tally_separator(const std::string& key,
			std::set<std::string>& separator) const;

	/**
	 * Return A*x
	 */
	Vector operator*(const VectorConfig& x) const;

	/**
	 * Return A^x
	 */
	VectorConfig operator^(const Vector& e) const;

	/**
	 * Return (dense) matrix associated with factor
	 * @param ordering of variables needed for matrix column order
	 * @param set weight to true to bake in the weights
	 */
	std::pair<Matrix, Vector> matrix(const Ordering& ordering, bool weight = true) const;

	/**
	 * Return (dense) matrix associated with factor
	 * The returned system is an augmented matrix: [A b]
	 * The standard deviations are NOT baked into A and b
	 * @param ordering of variables needed for matrix column order
	 */
	Matrix matrix_augmented(const Ordering& ordering) const;

	/**
	 * Return vectors i, j, and s to generate an m-by-n sparse matrix
	 * such that S(i(k),j(k)) = s(k), which can be given to MATLAB's sparse.
	 * As above, the standard deviations are baked into A and b
	 * @param first column index for each variable
	 */
	boost::tuple<std::list<int>, std::list<int>, std::list<double> >
		sparse(const Dimensions& columnIndices) const;

	/**
	 * Create a GaussianFactor on one variable 'alpha' (step size), in direction d
	 * @param x: starting point for search
	 * @param d: search direction
	 */
	shared_ptr alphaFactor(const VectorConfig& x, const VectorConfig& d) const;

	/* ************************************************************************* */
	// MUTABLE functions. FD:on the path to being eradicated
	/* ************************************************************************* */

	/** insert, copies A */
	void insert(const std::string& key, const Matrix& A) {
		As_.insert(std::make_pair(key, A));
	}

	/** set RHS, copies b */
	void set_b(const Vector& b) {
		this->b_ = b;
	}

	// set A matrices for the linear factor, same as insert ?
	inline void set_A(const std::string& key, const Matrix &A) {
		insert(key, A);
	}

	/**
	 * Current Implementation: Full QR factorization
	 * eliminate (in place!) one of the variables connected to this factor
	 * @param key the key of the node to be eliminated
	 * @return a new factor and a conditional gaussian on the eliminated variable
	 */
	std::pair<boost::shared_ptr<GaussianConditional>, shared_ptr>
	eliminate(const std::string& key) const;

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
