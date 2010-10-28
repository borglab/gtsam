/*
 * Preconditioner.h
 *
 *  Created on: Oct 27, 2010
 *      Author: Yong-Dian Jian
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/IterativeOptimizationParameters.h>

namespace gtsam {

template <class LINEAR, class VALUES>
class Preconditioner {

public:
	typedef IterativeOptimizationParameters Parameters;
	typedef boost::shared_ptr<Parameters> sharedParameters;
	typedef boost::shared_ptr<LINEAR> sharedLinear;
	typedef boost::shared_ptr<VALUES> sharedValues ;

public:
	Preconditioner(const LINEAR *ptr, const sharedParameters parameters, const sharedValues zeros):
		pLinear_(ptr), parameters_(parameters), zeros_(zeros){}

	void solve(const VALUES &values, VALUES &result) const { result = values ; }
	void solveTranspose(const VALUES &values, VALUES &result) const { result = values ; }

protected:
	const LINEAR* pLinear_ ;
	const sharedParameters parameters_ ;
	const sharedValues zeros_;

private:
	Preconditioner(){}
};

template <class LINEAR, class VALUES>
class JacobiPreconditioner : public Preconditioner<LINEAR,VALUES> {

public:

	typedef Preconditioner<LINEAR,VALUES> Base ;
	typedef IterativeOptimizationParameters Parameters;
	typedef boost::shared_ptr<Parameters> sharedParameters;
	typedef boost::shared_ptr<LINEAR> sharedLinear;
	typedef boost::shared_ptr<VALUES> sharedValues ;


	JacobiPreconditioner(const LINEAR *ptr, const sharedParameters parameters, const sharedValues zeros):
		Base(ptr, parameters, zeros), scale_(new VALUES(*zeros)) {
		initialize() ;
	}

	void initialize() {
		Base::pLinear_->getDiagonalOfHessian(*scale_);
		sqrt(*scale_) ;

		// check if any zero in scale_
		if ( anyZero(*scale_) ) {
			std::cout << "[JacobiPreconditioner] some diagonal element of hessian is zero" << std::endl ;
			throw std::runtime_error("JacobiPreconditioner");
		}
	}

	void solve(const VALUES &values, VALUES &result) const {
		//std::cout << "solve" << std::endl;
		ediv(values, *scale_, result) ;
	}

	void solveTranspose(const VALUES &values, VALUES &result) const {
		//std::cout << "solveTranspose" << std::endl;
		ediv(values, *scale_, result) ;
	}

protected:
	sharedValues scale_ ;

private:
	JacobiPreconditioner(){}
};

}
