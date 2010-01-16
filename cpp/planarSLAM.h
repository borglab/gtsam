/**
 *  @file  planarSLAM.h
 *  @brief: bearing/range measurements in 2D plane
 *  @authors Frank Dellaert
 **/

#pragma once

#include "Key.h"
#include "Pose2.h"
#include "Point2.h"
#include "NonlinearFactor.h"
#include "TupleConfig.h"
#include "NonlinearEquality.h"
#include "BetweenFactor.h"
#include "NonlinearFactorGraph.h"
#include "NonlinearOptimizer.h"

// We use gtsam namespace for generally useful factors
namespace gtsam {

	/**
	 * Binary factor for a bearing measurement
	 */
	template<class Config, class PoseKey, class PointKey>
	class BearingFactor: public NonlinearFactor2<Config, PoseKey, Pose2,
			PointKey, Point2> {
	private:

		Rot2 z_; /** measurement */

		typedef NonlinearFactor2<Config, PoseKey, Pose2, PointKey, Point2> Base;

	public:

		BearingFactor(); /* Default constructor */
		BearingFactor(const PoseKey& i, const PointKey& j, const Rot2& z,
				double sigma) :
			Base(sigma, i, j), z_(z) {
		}

		/** h(x)-z -> between(z,h(x)) for Rot2 manifold */
		Vector evaluateError(const Pose2& pose, const Point2& point,
				boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
			Rot2 hx = bearing(pose, point, H1, H2);
			return logmap(between(z_, hx));
		}
	}; // BearingFactor

	/**
	 * Binary factor for a range measurement
	 */
	template<class Config, class PoseKey, class PointKey>
	class RangeFactor: public NonlinearFactor2<Config, PoseKey, Pose2, PointKey,
			Point2> {
	private:

		double z_; /** measurement */

		typedef NonlinearFactor2<Config, PoseKey, Pose2, PointKey, Point2> Base;

	public:

		RangeFactor(); /* Default constructor */

		RangeFactor(const PoseKey& i, const PointKey& j, double z, double sigma) :
			Base(sigma, i, j), z_(z) {
		}

		/** h(x)-z */
		Vector evaluateError(const Pose2& pose, const Point2& point,
				boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
			double hx = gtsam::range(pose, point, H1, H2);
			return Vector_(1, hx - z_);
		}
	}; // RangeFactor

	// Use planarSLAM namespace for specific SLAM instance
	namespace planarSLAM {

		// Keys and Config
		typedef Symbol<Pose2, 'x'> PoseKey;
		typedef Symbol<Point2, 'l'> PointKey;
		typedef PairConfig<PoseKey, Pose2, PointKey, Point2> Config;

		// Factors
		typedef NonlinearEquality<Config, PoseKey, Pose2> Constraint;
	  typedef BetweenFactor<Config, PoseKey, Pose2> Odometry;
		typedef BearingFactor<Config, PoseKey, PointKey> Bearing;
		typedef RangeFactor<Config, PoseKey, PointKey> Range;

		// Graph
		struct Graph: public NonlinearFactorGraph<Config> {
			void addPoseConstraint(const PoseKey& i, const Pose2& p);
			void addOdometry(const PoseKey& i, const PoseKey& j, const Pose2& z, const Matrix& cov);
			void addBearing(const PoseKey& i, const PointKey& j, const Rot2& z, double sigma);
			void addRange(const PoseKey& i, const PointKey& j, double z, double sigma);
		};

		// Optimizer
		typedef NonlinearOptimizer<Graph, Config> Optimizer;

	} // planarSLAM

} // namespace gtsam

