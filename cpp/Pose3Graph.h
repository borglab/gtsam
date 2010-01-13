/**
 * @file    Pose3Graph.h
 * @brief   A factor graph for the 3D PoseSLAM problem
 * @author  Frank Dellaert
 * @author  Viorela Ila
 */

#pragma once

#include "Pose3Factor.h"
#include "Pose3Config.h"
#include "NonlinearFactorGraph.h"
#include "NonlinearEquality.h"

namespace gtsam {

	/**
	 * Non-linear factor graph for visual SLAM
	 */
	class Pose3Graph: public gtsam::NonlinearFactorGraph<Pose3Config> {

	public:

		/** default constructor is empty graph */
		Pose3Graph() {
		}

		/**
		 * equals
		 */
		bool equals(const Pose3Graph& p, double tol = 1e-9) const;

		/**
		 * Add a factor without having to do shared factor dance
		 */
		inline void add(const Pose3Config::Key& key1, const Pose3Config::Key& key2,
				const Pose3& measured, const Matrix& covariance) {
			push_back(sharedFactor(new Pose3Factor(key1, key2, measured, covariance)));
		}

		/**
		 *  Add an equality constraint on a pose
		 *  @param key of pose
		 *  @param pose which pose to constrain it to
		 */
		inline void addConstraint(const Pose3Config::Key& key, const Pose3& pose =Pose3()) {
			push_back(sharedFactor(new NonlinearEquality<Pose3Config,Pose3Config::Key,Pose3> (key, pose)));
		}

	private:
		/** Serialization function */
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version) {
		}
	};

} // namespace gtsam
