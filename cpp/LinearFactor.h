/**
 * @file    LinearFactor.h
 * @brief   Linear Factor....A Gaussian
 * @brief   linearFactor
 * @author  Christian Potthast
 */

// \callgraph

#pragma once

#include <list>
#include <map>
#include <set>
#include <ostream>
#include <boost/shared_ptr.hpp>

#include "Matrix.h"
#include "Factor.h"
#include "LinearFactorSet.h"
#include "ConditionalGaussian.h"
#include "Ordering.h"

#define CONSTRUCTOR 

namespace gtsam {

class MutableLinearFactor;

/**
 * Base Class for a linear factor.
 * LinearFactor is non-mutable (all methods const!).
 * The factor value is exp(-0.5*||Ax-b||^2)
 */
class LinearFactor: public Factor<FGConfig> {
public:

	typedef boost::shared_ptr<LinearFactor> shared_ptr;
	typedef std::map<std::string, Matrix>::iterator iterator;
	typedef std::map<std::string, Matrix>::const_iterator const_iterator;

protected:

	std::map<std::string, Matrix> As; // linear matrices
	Vector b; // right-hand-side

	LinearFactor()  {
	}

public:

	/** Construct Null factor */
	CONSTRUCTOR
	LinearFactor(const Vector& b_in) :
		b(b_in) { //TODO: add a way to initializing base class meaningfully
	}

	/** Construct unary factor */
	CONSTRUCTOR
	LinearFactor(const std::string& key1, const Matrix& A1, const Vector& b_in) :
		b(b_in) {
		As.insert(make_pair(key1, A1));
	}

	/** Construct binary factor */
	CONSTRUCTOR
	LinearFactor(const std::string& key1, const Matrix& A1,
			const std::string& key2, const Matrix& A2, const Vector& b_in) :
		b(b_in) {
		As.insert(make_pair(key1, A1));
		As.insert(make_pair(key2, A2));
	}

	/** Construct ternary factor */
	CONSTRUCTOR
	LinearFactor(const std::string& key1, const Matrix& A1,
			const std::string& key2, const Matrix& A2, const std::string& key3,
			const Matrix& A3, const Vector& b_in) :
		b(b_in) {
		As.insert(make_pair(key1, A1));
		As.insert(make_pair(key2, A2));
		As.insert(make_pair(key3, A3));
	}

	/** Construct from Conditional Gaussian */
	CONSTRUCTOR
	LinearFactor(const std::string& key, const boost::shared_ptr<
			ConditionalGaussian> cg) :
		b(cg->get_d()) {
		As.insert(make_pair(key, cg->get_R()));
		std::map<std::string, Matrix>::const_iterator it = cg->parentsBegin();
		for (; it != cg->parentsEnd(); it++) {
			const std::string& j = it->first;
			const Matrix& Aj = it->second;
			As.insert(make_pair(j, Aj));
		}
	}

	// Implementing Factor virtual functions

	double error(const FGConfig& c) const; /**  0.5*(A*x-b)'*(A*x-b) */
	void print(const std::string& s = "") const;
	bool equals(const LinearFactor& lf, double tol = 1e-9) const;
	std::string dump() const { return "";}
	std::size_t size() const { return As.size();}

	/** STL like, return the iterator pointing to the first node */
	const_iterator const begin() const { return As.begin();}

	/** STL like, return the iterator pointing to the last node */
	const_iterator const end() const { return As.end();	}

	/** check if empty */
	bool empty() const { return b.size() == 0;}

	/** get a copy of b */
	const Vector& get_b() const {	return b;	}

	/**
	 * get a copy of the A matrix from a specific node
	 * O(log n)
	 */
	const Matrix& get_A(const std::string& key) const {
		const_iterator it = As.find(key);
		if (it == As.end())
			throw(std::invalid_argument("LinearFactor::[] invalid key: " + key));
		return it->second;
	}

	/** operator[] syntax for get */
	inline const Matrix& operator[](const std::string& name) const {
		return get_A(name);
	}

	/** Check if factor involves variable with key */
	bool involves(const std::string& key) const {
		const_iterator it = As.find(key);
		return (it != As.end());
	}

	/**
	 * return the number of rows from the b vector
	 * @return a integer with the number of rows from the b vector
	 */
	int numberOfRows() const { return b.size();}

	/**
	 * Find all variables
	 * @return The set of all variable keys
	 */
	 std::list<std::string> keys() const;

	/**
	 * Find all variables and their dimensions
	 * @return The set of all variable/dimension pairs
	 */
	VariableSet variables() const;

	/**
	 * Add to separator set if this factor involves key, but don't add key itself
	 * @param key
	 * @param separator set to add to
	 */
	void tally_separator(const std::string& key,
			std::set<std::string>& separator) const;

	/**
	 * Return (dense) matrix associated with factor
	 * @param ordering of variables needed for matrix column order
	 */
	std::pair<Matrix, Vector> matrix(const Ordering& ordering) const;

}; // LinearFactor

/* ************************************************************************* */

/**
 * Mutable subclass of LinearFactor
 * To isolate bugs
 */
class MutableLinearFactor: public LinearFactor {
public:

	CONSTRUCTOR
	MutableLinearFactor() {
	}

	/**
	 * Constructor that combines a set of factors
	 * @param factors Set of factors to combine
	 */
	CONSTRUCTOR
	MutableLinearFactor(const std::set<shared_ptr> & factors);

	/** Construct unary mutable factor */
	CONSTRUCTOR
	MutableLinearFactor(const std::string& key1, const Matrix& A1,
			const Vector& b_in) :
				LinearFactor(key1, A1, b_in) {
	}

	/** Construct binary mutable factor */
	CONSTRUCTOR
	MutableLinearFactor(const std::string& key1, const Matrix& A1,
			const std::string& key2, const Matrix& A2, const Vector& b_in) :
				LinearFactor(key1, A1, key2, A2, b_in) {
	}

	/** Construct ternary mutable factor */
	CONSTRUCTOR
	MutableLinearFactor(const std::string& key1, const Matrix& A1,
			const std::string& key2, const Matrix& A2, const std::string& key3,
			const Matrix& A3, const Vector& b_in) :
				LinearFactor(key1, A1, key2, A2, key3, A3, b_in) {
	}

	/** insert, copies A */
	void insert(const std::string& key, const Matrix& A) {
		As.insert(std::make_pair(key, A));
	}

	/** set RHS, copies b */
	void set_b(const Vector& b) {
		this->b = b;
	}

	// set A matrices for the linear factor, same as insert ?
	inline void set_A(const std::string& key, const Matrix &A) {
		insert(key, A);
	}

	/**
	 * eliminate (in place!) one of the variables connected to this factor
	 * @param key the key of the node to be eliminated
	 * @return a new factor and a conditional gaussian on the eliminated variable
	 */
	std::pair<ConditionalGaussian::shared_ptr, shared_ptr> eliminate(const std::string& key);

	/**
	 * Take the factor f, and append to current matrices. Not very general.
	 * @param f linear factor graph
	 * @param m final number of rows of f, needs to be known in advance
	 * @param pos where to insert in the m-sized matrices
	 */
	void append_factor(LinearFactor::shared_ptr f, const size_t m,
			const size_t pos);

};

} // namespace gtsam
