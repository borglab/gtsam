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

 	template<class GRAPH, class LINEAR, class VALUES>
	class ConjugateGradientSolver : public IterativeSolver {

	protected:

		typedef boost::shared_ptr<GRAPH> sharedGRAPH ;
		typedef boost::shared_ptr<LINEAR> sharedLINEAR ;
		typedef boost::shared_ptr<VALUES> sharedVALUES ;
		typedef boost::shared_ptr<VectorValues> sharedVectorValues ;

		const LINEAR *ptr_;
		sharedVectorValues zeros_;

	public:

		typedef boost::shared_ptr<const ConjugateGradientSolver> shared_ptr ;

		// suggested constructor
		ConjugateGradientSolver(const GRAPH &graph, const VALUES &initial, const Ordering &ordering, const Parameters &parameters = Parameters()):
			IterativeSolver(parameters), ptr_(0), zeros_(boost::make_shared<VectorValues>(initial.zero(ordering))) {}

		// to be compatible to NonlinearOptimizer, but shouldn't be called...
		ConjugateGradientSolver(const LINEAR &GFG) {
			std::cout << "[ConjugateGradientSolver] Unexpected usage.." << std::endl;
			throw std::runtime_error("SubgraphSolver: gaussian factor graph initialization not supported");
	  	}

		// copy constructor
		ConjugateGradientSolver(const ConjugateGradientSolver& solver) : IterativeSolver(solver), ptr_(solver.ptr_), zeros_(solver.zeros_){}

		shared_ptr update(const LINEAR &graph) const {
			return boost::make_shared<ConjugateGradientSolver>(parameters_, &graph, zeros_) ;
		}

		VectorValues::shared_ptr optimize() const {
			VectorValues x = conjugateGradientDescent(*ptr_, *zeros_, *parameters_);
			return boost::make_shared<VectorValues>(x) ;
		}


		// for NonlinearOptimizer update only
		ConjugateGradientSolver(const sharedParameters parameters,
				                const LINEAR *ptr,
				                const sharedVectorValues zeros):
			IterativeSolver(parameters), ptr_(ptr), zeros_(zeros) {}

	private:
		// shouldn't be used
		ConjugateGradientSolver(){}
	};

 	template<class GRAPH, class LINEAR, class VALUES, class PC>
	class PreconditionedConjugateGradientSolver : public IterativeSolver {

	protected:

		typedef boost::shared_ptr<GRAPH> sharedGRAPH ;
		typedef boost::shared_ptr<LINEAR> sharedLINEAR ;
		typedef boost::shared_ptr<PC> sharedPreconditioner ;
		typedef boost::shared_ptr<VALUES> sharedVALUES ;
		typedef boost::shared_ptr<VectorValues> sharedVectorValues ;

		const LINEAR *ptr_;
		sharedPreconditioner pc_ ;
		sharedVectorValues zeros_;
	public:

		typedef boost::shared_ptr<const PreconditionedConjugateGradientSolver> shared_ptr ;

		// suggested constructor
		PreconditionedConjugateGradientSolver(
				const GRAPH &graph,
				const VALUES &initial,
				const Ordering &ordering,
				const Parameters &parameters = Parameters()):
			IterativeSolver(parameters), ptr_(0), pc_(), zeros_(boost::make_shared<VectorValues>(initial.zero(ordering))) {}

		// to be compatible to NonlinearOptimizer, but shouldn't be called...
		PreconditionedConjugateGradientSolver(const LINEAR &GFG) {
			std::cout << "[PreconditionedConjugateGradientSolver] Unexpected usage.." << std::endl;
			throw std::runtime_error("PreconditionedConjugateGradientSolver: gaussian factor graph initialization not supported");
	  	}

		// copy
		PreconditionedConjugateGradientSolver(const PreconditionedConjugateGradientSolver& solver) :
			IterativeSolver(solver), ptr_(solver.ptr_), pc_(solver.pc_), zeros_(solver.zeros_){}

		// called by NonlinearOptimizer
		shared_ptr update(const LINEAR &graph) const {

			return boost::make_shared<PreconditionedConjugateGradientSolver>(
					parameters_,
					&graph,
					boost::make_shared<PC>(&graph, parameters_, zeros_),
					zeros_) ;
		}

		// optimize with preconditioned conjugate gradient
		VectorValues::shared_ptr optimize() const {
			VectorValues x = preconditionedConjugateGradientDescent<LINEAR, PC, VectorValues>
					(*ptr_, *pc_, *zeros_, *parameters_);
			return boost::make_shared<VectorValues>(x) ;
		}

		PreconditionedConjugateGradientSolver(
				const sharedParameters parameters,
				const LINEAR *ptr,
				const sharedPreconditioner pc,
				const sharedVectorValues zeros):
			IterativeSolver(parameters), ptr_(ptr), pc_(pc), zeros_(zeros) {}
	private:

		// shouldn't be used
		PreconditionedConjugateGradientSolver(){}
	};


} // nsamespace gtsam
