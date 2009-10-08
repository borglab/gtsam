/*
 * LinearConstraint.cpp
 *
 *  Created on: Aug 10, 2009
 *      Author: alexgc
 */

#include <iostream>
#include <boost/foreach.hpp>
#include "LinearConstraint.h"
#include "Matrix.h"

namespace gtsam {
using namespace std;

LinearConstraint::LinearConstraint() {
}

LinearConstraint::LinearConstraint(const Vector& constraint,
		const std::string& id) :
	b(constraint) {
	int size = constraint.size();
	Matrix A = eye(size);
	As.insert(make_pair(id, A));
}

LinearConstraint::LinearConstraint(const std::string& node1, const Matrix& A1,
		const std::string& node2, const Matrix& A2, const Vector& rhs)
: b(rhs) {
	As.insert(make_pair(node1, A1));
	As.insert(make_pair(node2, A2));
}

LinearConstraint::LinearConstraint(const std::map<std::string, Matrix>& matrices, const Vector& rhs)
: As(matrices), b(rhs)
{
}

ConstrainedConditionalGaussian::shared_ptr LinearConstraint::eliminate(const std::string& key) {
	// check to ensure key is one of constraint nodes
	const_iterator it = As.find(key);
	if (it == As.end())
		throw invalid_argument("Node " + key + " is not in LinearConstraint");

	// extract the leading matrix
	Matrix A1 = it->second;

	// assemble parents
	map<string, Matrix> parents = As;
	parents.erase(key);

	// construct resulting CCG with parts
	ConstrainedConditionalGaussian::shared_ptr ccg(new ConstrainedConditionalGaussian(A1, parents, b));
	return ccg;
}

void LinearConstraint::print(const string& s) const {
	cout << s << ": " << dump() << endl;
}

bool LinearConstraint::equals(const LinearConstraint& f, double tol) const {
	// check sizes
	if (size() != f.size()) return false;

	// check rhs
	if (!equal_with_abs_tol(b, f.b, tol)) return false;

	// check all matrices
	pair<string, Matrix> p;
	BOOST_FOREACH(p, As) {
		// check for key existence
		const_iterator it = f.As.find(p.first);
		if (it == f.As.end()) return false;
		Matrix f_mat = it->second;
		// check matrix
		if (!(f_mat == p.second)) return false;
	}
	return true;
}

bool LinearConstraint::involves(const std::string& key) const {
	return As.find(key) != As.end();
}

list<string> LinearConstraint::keys(const std::string& key) const {
	list<string> ret;
	pair<string, Matrix> p;
	BOOST_FOREACH(p, As) {
		if (p.first != key)
			ret.push_back(p.first);
	}
	return ret;
}

string LinearConstraint::dump() const {
	string ret;
	return ret;
}

bool assert_equal(const LinearConstraint& actual,
		const LinearConstraint& expected, double tol) {
	bool ret = actual.equals(expected, tol);
	if (!ret) {
		cout << "Not Equal:" << endl;
		actual.print("Actual");
		expected.print("Expected");
	}
	return ret;
}

}
