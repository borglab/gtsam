/*
 * Pose2SLAMOptimizer.h
 *
 *  Created on: Jan 22, 2010
 *      Author: dellaert
 */

#ifndef POSE2SLAMOPTIMIZER_H_
#define POSE2SLAMOPTIMIZER_H_

#include <boost/foreach.hpp>

#include <gtsam/slam/pose2SLAM.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/linear/SubgraphSolver.h>

namespace gtsam {

	/**
	 * Optimizer class for use in MATLAB
	 * Keeps a Pose2Values estimate
	 * Returns all relevant matrices so MATLAB can optimize :-)
	 */
	class Pose2SLAMOptimizer {
	private:

		/** Non-linear factor graph */
		boost::shared_ptr<Pose2Graph> graph_;

		/** Current non-linear estimate */
		boost::shared_ptr<Pose2Values> theta_;

		/** Non-linear solver */
		typedef SubgraphSolver<Pose2Graph, Pose2Values> SPCG_Solver;
		SPCG_Solver solver_;

		/** Linear Solver */
		boost::shared_ptr<SubgraphPreconditioner> system_;

	public:
		/**
		 * Create optimizer: finds spanning tree and ordering
		 */
		Pose2SLAMOptimizer(const std::string& dataset = "",
				const std::string& path = "");

		/**
		 * print the object
		 */
		void print(const std::string& str = "") const;

		/**
		 * Virtual destructor
		 */
		virtual ~Pose2SLAMOptimizer() {
		}

		/**
		 * return graph pointer
		 */
		boost::shared_ptr<const Pose2Graph> graph() const {
			 return graph_;
		}

		/**
		 * return the current linearization point
		 */
		boost::shared_ptr<const Pose2Values> theta() const {
			return theta_;
		}

		/**
		 * linearize around current theta
		 */
		void linearize() {
			system_ = solver_.linearize(*graph_, *theta_);
		}

		/**
		 * Optimize to get a
		 */
		Vector optimize() const;

		double error() const;

		/**
		 * Return matrices associated with optimization problem
		 * around current non-linear estimate theta
		 * Returns [IJS] sparse representation
		 */
		Matrix a1() const { return system_->A1(*solver_.ordering()); }
		Matrix a2() const { return system_->A2(*solver_.ordering()); }
		Vector b1() const { return system_->b1(); }
		Vector b2() const { return system_->b2(); }
		std::pair<Matrix,Vector> Ab1() const { return system_->Ab1(*solver_.ordering()); }
		std::pair<Matrix,Vector> Ab2() const { return system_->Ab2(*solver_.ordering()); }

		/**
		 * update estimate with pure delta config x
		 */
		void update(const Vector& x);

		/**
		 * update estimate with pre-conditioned delta config y
		 */
		void updatePreconditioned(const Vector& y);

	};

}

#endif /* POSE2SLAMOPTIMIZER_H_ */
