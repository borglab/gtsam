/*
 * Pose2SLAMOptimizer.h
 *
 *  Created on: Jan 22, 2010
 *      Author: dellaert
 */

#ifndef POSE2SLAMOPTIMIZER_H_
#define POSE2SLAMOPTIMIZER_H_

#include "pose2SLAM.h"
#include "Ordering.h"
#include "SubgraphPreconditioner.h"

namespace gtsam {

	/**
	 * Optimizer class for use in MATLAB
	 * Keeps a Pose2Config estimate
	 * Returns all relevant matrices so MATLAB can optimize :-)
	 */
	class Pose2SLAMOptimizer {
	private:

		/** Non-linear factor graph */
		boost::shared_ptr<Pose2Graph> graph_;

		/** Current non-linear estimate */
		boost::shared_ptr<Pose2Config> theta_;

		/** Non-linear solver */
		typedef SubgraphPCG<Pose2Graph, Pose2Config> SPCG_Solver;
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
		 * linearize around current theta
		 */
		boost::shared_ptr<const Pose2Config> theta() const {
			return theta_;
		}
		/**
		 * linearize around current theta
		 */
		void linearize() {
			system_ = solver_.linearize(*graph_, *theta_);
		}

		/**
		 * update estimate with pure delta config x
		 */
		void update(const VectorConfig& x) {
			// TODO instead of assigning can we create a new one and replace the shared ptr ?
			*theta_ = expmap(*theta_, x);
			linearize();
		}

		/**
		 * Optimize to get a
		 */
		Vector optimize() {
			VectorConfig X = solver_.optimize(*system_);
			Vector x; // TODO convert to Vector
			return x;
		}

		/**
		 * Return matrices associated with optimization problem
		 * around current non-linear estimate theta
		 * Returns [IJS] sparse representation
		 */
		Matrix Ab1() const { return system_->Ab1(*solver_.ordering()); }
		Matrix Ab2() const { return system_->Ab2(*solver_.ordering()); }

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
