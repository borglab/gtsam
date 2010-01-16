/**
 * @file    Pose2Graph.h
 * @brief   A factor graph for the 2D PoseSLAM problem
 * @author  Frank Dellaert
 * @author  Viorela Ila
 */

#pragma once

#include "NonlinearFactorGraph.h"
#include "Pose2.h"
#include "LieConfig.h"
#include "BetweenFactor.h"
#include "Key.h"

namespace gtsam {

  /**
   * Pose2Config is now simply a typedef
   */
  typedef LieConfig<Symbol<Pose2,'x'>,Pose2> Pose2Config;

  typedef BetweenFactor<Pose2Config, Pose2Config::Key, Pose2> Pose2Factor;

  /**
   * Create a circle of n 2D poses tangent to circle of radius R, first pose at (R,0)
   * @param n number of poses
   * @param R radius of circle
   * @param c character to use for keys
   * @return circle of n 2D poses
   */
  Pose2Config pose2Circle(size_t n, double R);


	/**
	 * Non-linear factor graph for visual SLAM
	 */
	class Pose2Graph: public gtsam::NonlinearFactorGraph<Pose2Config> {

	public:

		/** default constructor is empty graph */
		Pose2Graph() {
		}

		/**
		 * equals
		 */
		bool equals(const Pose2Graph& p, double tol = 1e-9) const;

		/**
		 * Add a factor without having to do shared factor dance
		 */
		inline void add(const Pose2Config::Key& key1, const Pose2Config::Key& key2,
				const Pose2& measured, const Matrix& covariance) {
			push_back(sharedFactor(new Pose2Factor(key1, key2, measured, covariance)));
		}

		/**
		 *  Add an equality constraint on a pose
		 *  @param key of pose
		 *  @param pose which pose to constrain it to
		 */
		void addConstraint(const Pose2Config::Key& key, const Pose2& pose =	Pose2());

	private:
		/** Serialization function */
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version) {
		}
	};

} // namespace gtsam
