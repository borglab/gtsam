/**
 * @file    simulated2D.h
 * @brief   measurement functions and derivatives for simulated 2D robot
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

#include "Pose2.h"
#include "TupleConfig.h"
#include "NonlinearFactor.h"

// \namespace

namespace gtsam {

	namespace simulated2DOriented {

		// The types that take an oriented pose2 rather than point2
		typedef TypedSymbol<Point2, 'l'> PointKey;
		typedef TypedSymbol<Pose2,  'x'> PoseKey;
		typedef PairConfig<PoseKey, Pose2,  PointKey, Point2> Config;

		//TODO:: point prior is not implemented right now

		/**
		 * Prior on a single pose, and optional derivative version
		 */
		inline Pose2 prior(const Pose2& x) {
			return x;
		}
		Pose2 prior(const Pose2& x, boost::optional<Matrix&> H = boost::none);

		/**
		 * Unary factor encoding a soft prior on a vector
		 */
		template<class Cfg = Config, class Key = PoseKey>
		struct GenericPosePrior: public NonlinearFactor1<Cfg, Key, Point2> {

			Pose2 z_;

			GenericPosePrior(const Pose2& z, const SharedGaussian& model, const Key& key) :
				NonlinearFactor1<Cfg, Key, Point2> (model, key), z_(z) {
			}

			Vector evaluateError(const Pose2& x, boost::optional<Matrix&> H =
					boost::none) const {
				return logmap(z_, prior(x, H));
			}

		};

	} // namespace simulated2DOriented
} // namespace gtsam
