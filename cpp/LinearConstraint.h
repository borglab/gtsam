/**
 * @file LinearConstraint.h
 * @brief Class that implements linear equality constraints
 * @author Alex Cunningham
 */

#ifndef EQUALITYFACTOR_H_
#define EQUALITYFACTOR_H_

#include <list>
#include <set>
#include "Matrix.h"
#include "ConstrainedConditionalGaussian.h"

namespace gtsam {

/**
 * Linear constraints are similar to factors in structure, but represent
 * a different problem
 */
class LinearConstraint: Testable<LinearConstraint> {
public:

	typedef boost::shared_ptr<LinearConstraint> shared_ptr;
	typedef std::map<std::string, Matrix>::iterator iterator;
	typedef std::map<std::string, Matrix>::const_iterator const_iterator;

protected:
	std::map<std::string, Matrix> As; // linear matrices
	Vector b; // right-hand-side

public:
	/**
	 * Default constructor
	 */
	LinearConstraint();

	/**
	 * Constructor with initialization of a unary equality factor
	 * Creates an identity matrix for the underlying implementation and the constraint
	 * value becomes the RHS value - use for setting a variable to a fixed value
	 * @param constraint the value that the variable node is defined as equal to
	 * @param key identifier for the variable node
	 */
	LinearConstraint(const Vector& constraint, const std::string& key);

	/**
	 * Constructor for binary constraint
	 * @param key for first node
	 * @param A Matrix for first node
	 * @param key for second node
	 * @param A Matrix for second node
	 * @param RHS b vector
	 */
	LinearConstraint(const std::string& node1, const Matrix& A1,
			const std::string& node2, const Matrix& A2, const Vector& b);

	/**
	 * Constructor for arbitrary numbers of nodes
	 * @param matrices is the full map of A matrices
	 * @param b is the RHS vector
	 */
	LinearConstraint(const std::map<std::string, Matrix>& matrices,
			const Vector& b);

	/**
	 * Default Destructor
	 */
	~LinearConstraint() {
	}

	/**
	 * print
	 * @param s optional string naming the factor
	 */
	void print(const std::string& s = "") const;

	/**
	 * equality up to tolerance
	 */
	bool equals(const LinearConstraint& f, double tol = 1e-9) const;

	/**
	 * Eliminates the constraint
	 * Note:  Assumes that the constraint will be completely eliminated
	 * and that the matrix associated with the key is invertible
	 * @param key is the variable to eliminate
	 * @return a constrained conditional gaussian for the variable that is a
	 * function of its parents
	 */
	ConstrainedConditionalGaussian::shared_ptr
	eliminate(const std::string& key);

	/** get a copy of b */
	const Vector& get_b() const {
		return b;
	}

	/** check if the constraint is connected to a particular node */
	bool involves(const std::string& key) const;

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

	/**
	 * Gets all of the keys connected in a constraint
	 * @param key is a key to leave out of the final set
	 * @return a list of the keys for nodes connected to the constraint
	 */
	std::list<std::string> keys(const std::string& key = "") const;

	/**
	 * @return the number of nodes the constraint connects
	 */
	std::size_t size() const {
		return As.size();
	}

	// friends
	friend LinearConstraint::shared_ptr combineConstraints(const std::set<LinearConstraint::shared_ptr>& constraints);
};

/**
 * Combines constraints into one constraint
 */
LinearConstraint::shared_ptr combineConstraints(const std::set<LinearConstraint::shared_ptr>& constraints);

}

#endif /* EQUALITYFACTOR_H_ */
