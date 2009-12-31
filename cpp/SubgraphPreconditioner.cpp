/*
 * SubgraphPreconditioner.cpp
 * Created on: Dec 31, 2009
 * @author: Frank Dellaert
 */

#include <boost/foreach.hpp>
#include "SubgraphPreconditioner.h"

using namespace std;

namespace gtsam {

	/* ************************************************************************* */
	SubgraphPreconditioner::SubgraphPreconditioner(const GaussianBayesNet& Rc1,
			const GaussianFactorGraph& Ab2, const VectorConfig& xbar) :
		Rc1_(Rc1), Ab2_(Ab2), xbar_(xbar), b2bar_(Ab2_.errors(xbar)) {
	}

	/* ************************************************************************* */
	// x = xbar + inv(R1)*y
	VectorConfig SubgraphPreconditioner::x(const VectorConfig& y) const {
		return xbar_ + gtsam::backSubstitute(Rc1_, y);
	}

	/* ************************************************************************* */
	double SubgraphPreconditioner::error(const VectorConfig& y) const {

		Errors e;

		// Use BayesNet order to add y contributions in order
		BOOST_FOREACH(GaussianConditional::shared_ptr cg, Rc1_) {
			const string& j = cg->key();
			e.push_back(y[j]); // append y
		}

		// Add A2 contribution
		VectorConfig x = this->x(y);
		Errors e2 = Ab2_.errors(x);
		e.splice(e.end(), e2);

		return 0.5 * dot(e, e);
	}

	/* ************************************************************************* */
	// gradient is y + inv(R1')*A2'*(A2*inv(R1)*y-b2bar),
	VectorConfig SubgraphPreconditioner::gradient(const VectorConfig& y) const {
		VectorConfig x = this->x(y); // x = inv(R1)*y
		VectorConfig gx2 = Ab2_ ^ Ab2_.errors(x);
		VectorConfig gy2 = gtsam::backSubstituteTranspose(Rc1_, gx2); // inv(R1')*gx2
		return y + gy2;
	}

	/* ************************************************************************* */
	// Apply operator A, A*y = [I;A2*inv(R1)]*y = [y; A2*inv(R1)*y]
	Errors SubgraphPreconditioner::operator*(const VectorConfig& y) const {

		Errors e;

		// Use BayesNet order to add y contributions in order
		BOOST_FOREACH(GaussianConditional::shared_ptr cg, Rc1_) {
			const string& j = cg->key();
			e.push_back(y[j]); // append y
		}

		// Add A2 contribution
		VectorConfig x = gtsam::backSubstitute(Rc1_, y); // x=inv(R1)*y
		Errors e2 = Ab2_ * x; // A2*x
		e.splice(e.end(), e2);

		return e;
	}

	/* ************************************************************************* */
	// Apply operator A', A'*e = [I inv(R1')*A2']*e = e1 + inv(R1')*A2'*e2
	VectorConfig SubgraphPreconditioner::operator^(const Errors& e) const {

		VectorConfig y1;

		// Use BayesNet order to remove y contributions in order
		Errors::const_iterator it = e.begin();
		BOOST_FOREACH(GaussianConditional::shared_ptr cg, Rc1_) {
			const string& j = cg->key();
			const Vector& ej = *(it++);
			y1.insert(j,ej);
		}

		// create e2 with what's left of e
		Errors e2;
		while (it != e.end())
		e2.push_back(*(it++));

		// get A2 part,
		VectorConfig x = Ab2_ ^ e2; // x = A2'*e2
		VectorConfig y2 = gtsam::backSubstituteTranspose(Rc1_, x); // inv(R1')*x;

		return y1 + y2;
	}
	/* ************************************************************************* */

} // nsamespace gtsam
