/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    simulated2D.h
 * @brief   measurement functions and derivatives for simulated 2D robot
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/TupleValues.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

// \namespace

namespace gtsam {

	namespace simulated2DOriented {

		// The types that take an oriented pose2 rather than point2
		typedef TypedSymbol<Point2, 'l'> PointKey;
		typedef TypedSymbol<Pose2,  'x'> PoseKey;
		typedef LieValues<PoseKey> PoseValues;
		typedef LieValues<PointKey> PointValues;
		typedef TupleValues2<PoseValues, PointValues> Values;

		//TODO:: point prior is not implemented right now

		/**
		 * Prior on a single pose, and optional derivative version
		 */
		inline Pose2 prior(const Pose2& x) {
			return x;
		}
		Pose2 prior(const Pose2& x, boost::optional<Matrix&> H = boost::none);

		/**
		 * odometry between two poses, and optional derivative version
		 */
		inline Pose2 odo(const Pose2& x1, const Pose2& x2) {
			return x1.between(x2);
		}
		Pose2 odo(const Pose2& x1, const Pose2& x2, boost::optional<Matrix&> H1 =
				boost::none, boost::optional<Matrix&> H2 = boost::none);

		/**
		 * Unary factor encoding a soft prior on a vector
		 */
		template<class Cfg = Values, class Key = PoseKey>
		struct GenericPosePrior: public NonlinearFactor1<Cfg, Key> {

			Pose2 z_;

			GenericPosePrior(const Pose2& z, const SharedGaussian& model, const Key& key) :
				NonlinearFactor1<Cfg, Key> (model, key), z_(z) {
			}

			Vector evaluateError(const Pose2& x, boost::optional<Matrix&> H =
					boost::none) const {
				return z_.logmap(prior(x, H));
			}

		};

		/**
		 * Binary factor simulating "odometry" between two Vectors
		 */
		template<class Cfg = Values, class Key = PoseKey>
		struct GenericOdometry: public NonlinearFactor2<Cfg, Key, Key> {
			Pose2 z_;

			GenericOdometry(const Pose2& z, const SharedGaussian& model,
					const Key& i1, const Key& i2) :
				NonlinearFactor2<Cfg, Key, Key> (model, i1, i2), z_(z) {
			}

			Vector evaluateError(const Pose2& x1, const Pose2& x2, boost::optional<
					Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none) const {
				return z_.logmap(odo(x1, x2, H1, H2));
			}

		};

		typedef GenericOdometry<Values, PoseKey> Odometry;

	} // namespace simulated2DOriented
} // namespace gtsam
