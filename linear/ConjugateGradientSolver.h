/*
 * ConjugateGradientSolver.h
 *
 *  Created on: Oct 21, 2010
 *      Author: Yong-Dian Jian
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <gtsam/linear/IterativeSolver.h>
#include <gtsam/linear/iterative-inl.h>
#include <gtsam/nonlinear/Ordering.h>

namespace gtsam {

	// GaussianFactorGraph, Values
 	template<class GRAPH, class LINEAR, class VALUES>
	class ConjugateGradientSolver : public IterativeSolver {

	protected:
		const LINEAR *ptr_;

		typedef boost::shared_ptr<VectorValues> sharedVectorValues ;
		sharedVectorValues zeros_;

	public:

		typedef boost::shared_ptr<const ConjugateGradientSolver> shared_ptr ;

		ConjugateGradientSolver(const GRAPH &graph, const VALUES &initial, const Ordering &ordering, const sharedParameters parameters):
			IterativeSolver(parameters), ptr_(0), zeros_(boost::make_shared<VectorValues>(initial.zero(ordering))) {}

		ConjugateGradientSolver(const LINEAR &GFG) {
			throw std::runtime_error("SubgraphSolver: gaussian factor graph initialization not supported");
	  	}

		ConjugateGradientSolver(const ConjugateGradientSolver& solver) : IterativeSolver(solver), ptr_(solver.ptr_), zeros_(solver.zeros_){}
		ConjugateGradientSolver(const sharedParameters parameters, const LINEAR *ptr, const sharedVectorValues zeros):
			IterativeSolver(parameters), ptr_(ptr), zeros_(zeros) {}

		shared_ptr update(const LINEAR &graph) const {
			return boost::make_shared<ConjugateGradientSolver>(parameters_, &graph, zeros_) ;
		}

		VectorValues::shared_ptr optimize() const {
			//boost::shared_ptr<VectorValues> zeros (ptr_->allocateVectorVavlues());
			VectorValues x = conjugateGradientDescent(*ptr_, *zeros_, parameters_);
			return boost::make_shared<VectorValues>(x) ;
		}

	private:
		ConjugateGradientSolver():IterativeSolver(),ptr_(0){}

	};

} // nsamespace gtsam
