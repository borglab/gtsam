/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * SubgraphPreconditioner.cpp
 * Created on: Dec 31, 2009
 * @author: Frank Dellaert
 */

#include <boost/foreach.hpp>
#include <gtsam/linear/SubgraphPreconditioner.h>

using namespace std;

namespace gtsam {

	/* ************************************************************************* */
	SubgraphPreconditioner::SubgraphPreconditioner(sharedFG& Ab1, sharedFG& Ab2,
			sharedBayesNet& Rc1, sharedValues& xbar) :
		Ab1_(Ab1), Ab2_(Ab2), Rc1_(Rc1), xbar_(xbar), b2bar_(Ab2_->errors_(*xbar)) {
	}

	/* ************************************************************************* */
	// x = xbar + inv(R1)*y
	VectorValues SubgraphPreconditioner::x(const VectorValues& y) const {
#ifdef VECTORBTREE
		if (!y.cloned(*xbar_)) throw
			invalid_argument("SubgraphPreconditioner::x: y needs to be cloned from xbar");
#endif
		VectorValues x = y;
		backSubstituteInPlace(*Rc1_,x);
		x += *xbar_;
		return x;
	}

	/* ************************************************************************* */
	double SubgraphPreconditioner::error(const VectorValues& y) const {

		Errors e;

		// Use BayesNet order to add y contributions in order
		BOOST_FOREACH(GaussianConditional::shared_ptr cg, *Rc1_) {
			const Symbol& j = cg->key();
			e.push_back(y[j]); // append y
		}

		// Add A2 contribution
		VectorValues x = this->x(y);
		Errors e2 = Ab2_->errors(x);
		e.splice(e.end(), e2);

		return 0.5 * dot(e, e);
	}

	/* ************************************************************************* */
	// gradient is y + inv(R1')*A2'*(A2*inv(R1)*y-b2bar),
	VectorValues SubgraphPreconditioner::gradient(const VectorValues& y) const {
		VectorValues x = this->x(y); // x = inv(R1)*y
		Errors e2 = Ab2_->errors(x);
		VectorValues gx2 = VectorValues::zero(y);
		Ab2_->transposeMultiplyAdd(1.0,e2,gx2); // A2'*e2;
		VectorValues gy2 = gtsam::backSubstituteTranspose(*Rc1_, gx2); // inv(R1')*gx2
		return y + gy2;
	}

	/* ************************************************************************* */
	// Apply operator A, A*y = [I;A2*inv(R1)]*y = [y; A2*inv(R1)*y]
	Errors SubgraphPreconditioner::operator*(const VectorValues& y) const {

		Errors e;

		// Use BayesNet order to add y contributions in order
		BOOST_FOREACH(GaussianConditional::shared_ptr cg, *Rc1_) {
			const Symbol& j = cg->key();
			e.push_back(y[j]); // append y
		}

		// Add A2 contribution
		VectorValues x = y; // TODO avoid ?
		gtsam::backSubstituteInPlace(*Rc1_, x); // x=inv(R1)*y
		Errors e2 = *Ab2_ * x; // A2*x
		e.splice(e.end(), e2);

		return e;
	}

	/* ************************************************************************* */
	// In-place version that overwrites e
	void SubgraphPreconditioner::multiplyInPlace(const VectorValues& y, Errors& e) const {

		Errors::iterator ei = e.begin();

		// Use BayesNet order to add y contributions in order
		BOOST_FOREACH(GaussianConditional::shared_ptr cg, *Rc1_) {
			const Symbol& j = cg->key();
			*ei = y[j]; // append y
			ei++;
		}

		// Add A2 contribution
		VectorValues x = y; // TODO avoid ?
		gtsam::backSubstituteInPlace(*Rc1_, x); // x=inv(R1)*y
		Ab2_->multiplyInPlace(x,ei); // use iterator version
	}

	/* ************************************************************************* */
	// Apply operator A', A'*e = [I inv(R1')*A2']*e = e1 + inv(R1')*A2'*e2
	VectorValues SubgraphPreconditioner::operator^(const Errors& e) const {

		VectorValues y;

		// Use BayesNet order to remove y contributions in order
		Errors::const_iterator it = e.begin();
		BOOST_FOREACH(GaussianConditional::shared_ptr cg, *Rc1_) {
			const Symbol& j = cg->key();
			const Vector& ej = *(it++);
			y.insert(j,ej);
		}

		// get A2 part
		transposeMultiplyAdd2(1.0,it,e.end(),y);

		return y;
	}

	/* ************************************************************************* */
	// y += alpha*A'*e
	void SubgraphPreconditioner::transposeMultiplyAdd
		(double alpha, const Errors& e, VectorValues& y) const {

		// Use BayesNet order to remove y contributions in order
		Errors::const_iterator it = e.begin();
		BOOST_FOREACH(GaussianConditional::shared_ptr cg, *Rc1_) {
			const Symbol& j = cg->key();
			const Vector& ej = *(it++);
			axpy(alpha,ej,y[j]);
		}

		// get A2 part
		transposeMultiplyAdd2(alpha,it,e.end(),y);
	}

	/* ************************************************************************* */
	// y += alpha*inv(R1')*A2'*e2
	void SubgraphPreconditioner::transposeMultiplyAdd2 (double alpha,
		Errors::const_iterator it, Errors::const_iterator end, VectorValues& y) const {

		// create e2 with what's left of e
		// TODO can we avoid creating e2 by passing iterator to transposeMultiplyAdd ?
		Errors e2;
		while (it != end)
		e2.push_back(*(it++));

		// Old code:
		// VectorValues x = *Ab2_ ^ e2; // x = A2'*e2
		// y += alpha * gtsam::backSubstituteTranspose(*Rc1_, x); // inv(R1')*x;
		// New Code:
		VectorValues x = VectorValues::zero(y); // x = 0
		Ab2_->transposeMultiplyAdd(1.0,e2,x);   // x += A2'*e2
		axpy(alpha, gtsam::backSubstituteTranspose(*Rc1_, x), y); // y += alpha*inv(R1')*x
	}

	/* ************************************************************************* */
	void SubgraphPreconditioner::print(const std::string& s) const {
		cout << s << endl;
		Ab2_->print();
	}
} // nsamespace gtsam
