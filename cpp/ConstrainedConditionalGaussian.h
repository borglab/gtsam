/**
 * @file ConstrainedConditionalGaussian.h
 * @brief Class which implements a conditional gaussian that handles situations
 * that occur when linear constraints appear in the system.  A constrained
 * conditional gaussian is the result of eliminating a linear equality
 * constraint.
 *
 *  @author Alex Cunningham
 */

#pragma once

#include "ConditionalGaussian.h"

namespace gtsam {

/**
 * Implements a more generalized conditional gaussian to represent
 * the result of eliminating an equality constraint.  All of the
 * forms of an equality constraint will be of the form
 * A1x = b - sum(Aixi from i=2 to i=N)
 * If A1 is triangular, then it can be solved using backsubstitution
 * If A1 is invertible, then it can be solved with the Matrix::solve() command
 * If A1 overconstrains the system, then ???
 * If A1 underconstrains the system, then ???
 */
class ConstrainedConditionalGaussian: public ConditionalGaussian {

public:
	typedef boost::shared_ptr<ConstrainedConditionalGaussian> shared_ptr;

	/**
	 * Default Constructor
	 * Don't use this
	 */
	ConstrainedConditionalGaussian(const std::string& key);

	/**
	 * Used for unary factors that simply associate a name with a particular value
	 * Can use backsubstitution to solve trivially
	 * @param value is a fixed value for x in the form x = value
	 */
	ConstrainedConditionalGaussian(const std::string& key, const Vector& value);

	/**
	 * Used for unary factors of the form Ax=b
	 * Invertability of A is significant
	 * @param b is the RHS of the equation
	 * @param A is the A matrix
	 */
	ConstrainedConditionalGaussian(const std::string& key, const Vector& value, const Matrix& A);

	/**
	 * Binary constructor of the form A1*x = b - A2*y
	 * for node with one parent
	 * @param b is the RHS of the equation
	 * @param A1 is the A1 matrix
	 * @param parent is the string identifier for the parent node
	 * @param A2 is the A2 matrix
	 */
	ConstrainedConditionalGaussian(const std::string& key, const Vector& b, const Matrix& A1,
			const std::string& parent, const Matrix& A2);

	/**
	 * Ternary constructor of the form A1*x = b - A2*y - A3*z
	 * @param b is the RHS of the equation
	 * @param A1 is the A1 matrix
	 * @param parentY string id for y
	 * @param A2 is the A2 matrix
	 * @param parentZ string id for z
	 * @param A3 is the A3 matrix
	 */
	ConstrainedConditionalGaussian(const std::string& key, const Vector& b, const Matrix& A1,
			const std::string& parentY, const Matrix& A2,
			const std::string& parentZ, const Matrix& A3);

	/**
	 * Construct with arbitrary number of parents of form
	 * A1*x = b - sum(Ai*xi)
	 * @param A1 is the matrix associated with the variable that was eliminated
	 * @param parents is the map of parents (Ai and xi from above)
	 * @param b is the rhs vector
	 */
	ConstrainedConditionalGaussian(const std::string& key, const Matrix& A1,
			const std::map<std::string, Matrix>& parents, const Vector& b);

	virtual ~ConstrainedConditionalGaussian() {
	}

	/**
	 * Solve for the value of the node given the parents
	 * If A1 (R from the conditional gaussian) is triangular, then backsubstitution
	 * If A1 invertible, Matrix::solve()
	 * If A1 under/over constrains the system, TODO
	 * @param config contains the values for all of the parents
	 * @return the value for this node
	 */
	Vector solve(const VectorConfig& x) const;

};

}
