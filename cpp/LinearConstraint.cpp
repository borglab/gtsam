/**
 * @file LinearConstraint.cpp
 * @author Alex Cunningham
 */

#include <iostream>
#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include "LinearConstraint.h"
#include "Matrix.h"

namespace gtsam {
using namespace std;

// trick from some reading group
#define FOREACH_PAIR( KEY, VAL, COL) BOOST_FOREACH (boost::tie(KEY,VAL),COL)

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

void LinearConstraint::print(const string& s) const {
	cout << s << ": " << endl;
	string key; Matrix A;
	FOREACH_PAIR(key, A, As) {
		gtsam::print(A, key);
	}
	gtsam::print(b, "rhs= ");
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
		if (!equal_with_abs_tol(f_mat,p.second,tol)) return false;
	}
	return true;
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

LinearConstraint::shared_ptr combineConstraints(
		const set<LinearConstraint::shared_ptr>& constraints) {
	map<string, Matrix> blocks;
	Vector rhs;
	BOOST_FOREACH(LinearConstraint::shared_ptr c, constraints) {
		string key; Matrix A;
		FOREACH_PAIR( key, A, c->As) {
			if (blocks.find(key) == blocks.end())
				blocks[key] = A;
			else {
				Matrix Aold = blocks[key];
				if (A.size1() != Aold.size1() || A.size2() != Aold.size2())
					throw invalid_argument("Dimension mismatch");
				blocks[key] = A + Aold;
			}
		}

		// assemble rhs
		if (rhs.size() == 0)
			rhs = c->get_b();
		else
			rhs = rhs + c->get_b();
	}
	LinearConstraint::shared_ptr result(new LinearConstraint(blocks, rhs));
	return result;
}

}
