/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SubgraphPreconditioner.cpp
 * @date Dec 31, 2009
 * @author: Frank Dellaert
 */

#include <boost/foreach.hpp>
#include <gtsam/linear/SubgraphPreconditioner.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/inference/FactorGraph-inl.h>

using namespace std;

namespace gtsam {

	/* ************************************************************************* */
	SubgraphPreconditioner::SubgraphPreconditioner(const sharedFG& Ab1, const sharedFG& Ab2,
			const sharedBayesNet& Rc1, const sharedValues& xbar) :
		Ab1_(Ab1), Ab2_(Ab2), Rc1_(Rc1), xbar_(xbar), b2bar_(gaussianErrors_(*Ab2_,*xbar)) {
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

//    SubgraphPreconditioner SubgraphPreconditioner::add_priors(double sigma) const {
//    	SubgraphPreconditioner result = *this ;
//    	result.Ab2_ = sharedFG(new GaussianFactorGraph(Ab2_->add_priors(sigma))) ;
//    	return result ;
//    }

	/* ************************************************************************* */
	double error(const SubgraphPreconditioner& sp, const VectorValues& y) {

		Errors e(y);
		VectorValues x = sp.x(y);
		Errors e2 = gaussianErrors(*sp.Ab2(),x);
		return 0.5 * (dot(e, e) + dot(e2,e2));
	}

	/* ************************************************************************* */
	// gradient is y + inv(R1')*A2'*(A2*inv(R1)*y-b2bar),
	VectorValues gradient(const SubgraphPreconditioner& sp, const VectorValues& y) {
		VectorValues x = sp.x(y); // x = inv(R1)*y
		Errors e2 = gaussianErrors(*sp.Ab2(),x);
		VectorValues gx2 = VectorValues::Zero(y);
		gtsam::transposeMultiplyAdd(*sp.Ab2(),1.0,e2,gx2); // A2'*e2;
		VectorValues gy2 = gtsam::backSubstituteTranspose(*sp.Rc1(), gx2); // inv(R1')*gx2
		return y + gy2;
	}

	/* ************************************************************************* */
	// Apply operator A, A*y = [I;A2*inv(R1)]*y = [y; A2*inv(R1)*y]
	Errors operator*(const SubgraphPreconditioner& sp, const VectorValues& y) {

		Errors e(y);

		// Add A2 contribution
		VectorValues x = y; // TODO avoid ?
		gtsam::backSubstituteInPlace(*sp.Rc1(), x); // x=inv(R1)*y
		Errors e2 = *sp.Ab2() * x; // A2*x
		e.splice(e.end(), e2);

		return e;
	}

	/* ************************************************************************* */
	// In-place version that overwrites e
	void multiplyInPlace(const SubgraphPreconditioner& sp, const VectorValues& y, Errors& e) {


		Errors::iterator ei = e.begin();
		for ( Index i = 0 ; i < y.size() ; ++i, ++ei ) {
			*ei = y[i];
		}

		// Add A2 contribution
		VectorValues x = y; // TODO avoid ?
		gtsam::backSubstituteInPlace(*sp.Rc1(), x); // x=inv(R1)*y
		gtsam::multiplyInPlace(*sp.Ab2(),x,ei); // use iterator version
	}

	/* ************************************************************************* */
	// Apply operator A', A'*e = [I inv(R1')*A2']*e = e1 + inv(R1')*A2'*e2
	VectorValues operator^(const SubgraphPreconditioner& sp, const Errors& e) {

		Errors::const_iterator it = e.begin();
		VectorValues y = sp.zero();
		for ( Index i = 0 ; i < y.size() ; ++i, ++it )
			y[i] = *it ;
		sp.transposeMultiplyAdd2(1.0,it,e.end(),y);
		return y;
	}

	/* ************************************************************************* */
	// y += alpha*A'*e
	void transposeMultiplyAdd
		(const SubgraphPreconditioner& sp, double alpha, const Errors& e, VectorValues& y) {


		Errors::const_iterator it = e.begin();
		for ( Index i = 0 ; i < y.size() ; ++i, ++it ) {
			const Vector& ei = *it;
			axpy(alpha,ei,y[i]);
		}
		sp.transposeMultiplyAdd2(alpha,it,e.end(),y);
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

		VectorValues x = VectorValues::Zero(y); // x = 0
		gtsam::transposeMultiplyAdd(*Ab2_,1.0,e2,x);   // x += A2'*e2
		axpy(alpha, gtsam::backSubstituteTranspose(*Rc1_, x), y); // y += alpha*inv(R1')*x
	}

	/* ************************************************************************* */
	void SubgraphPreconditioner::print(const std::string& s) const {
		cout << s << endl;
		Ab2_->print();
	}
} // nsamespace gtsam
