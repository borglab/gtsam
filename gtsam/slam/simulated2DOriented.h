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
		typedef TypedSymbol<Pose2, 'x'> PoseKey;
		typedef Values<PoseKey> PoseValues;
		typedef Values<PointKey> PointValues;
		typedef TupleValues2<PoseValues, PointValues> Values;

		//TODO:: point prior is not implemented right now

		/// Prior on a single pose
		inline Pose2 prior(const Pose2& x) {
			return x;
		}

		/// Prior on a single pose, optional derivative version
		Pose2 prior(const Pose2& x, boost::optional<Matrix&> H = boost::none);

		/// odometry between two poses
		inline Pose2 odo(const Pose2& x1, const Pose2& x2) {
			return x1.between(x2);
		}

		/// odometry between two poses, optional derivative version
		Pose2 odo(const Pose2& x1, const Pose2& x2, boost::optional<Matrix&> H1 =
				boost::none, boost::optional<Matrix&> H2 = boost::none);

		/// Unary factor encoding a soft prior on a vector
		template<class VALUES = Values, class Key = PoseKey>
		struct GenericPosePrior: public NonlinearFactor1<VALUES, Key> {

			Pose2 z_; ///< measurement

			/// Create generic pose prior
			GenericPosePrior(const Pose2& z, const SharedNoiseModel& model,
					const Key& key) :
					NonlinearFactor1<VALUES, Key>(model, key), z_(z) {
			}

			/// Evaluate error and optionally derivative
			Vector evaluateError(const Pose2& x, boost::optional<Matrix&> H =
					boost::none) const {
				return z_.localCoordinates(prior(x, H));
			}

		};

		/**
		 * Binary factor simulating "odometry" between two Vectors
		 */
		template<class VALUES = Values, class KEY = PoseKey>
		struct GenericOdometry: public NonlinearFactor2<VALUES, KEY, KEY> {
			Pose2 z_;   ///< Between measurement for odometry factor

			/**
			 * Creates an odometry factor between two poses
			 */
			GenericOdometry(const Pose2& z, const SharedNoiseModel& model,
					const KEY& i1, const KEY& i2) :
					NonlinearFactor2<VALUES, KEY, KEY>(model, i1, i2), z_(z) {
			}

			/// Evaluate error and optionally derivative
			Vector evaluateError(const Pose2& x1, const Pose2& x2,
					boost::optional<Matrix&> H1 = boost::none,
					boost::optional<Matrix&> H2 = boost::none) const {
				return z_.localCoordinates(odo(x1, x2, H1, H2));
			}

		};

		typedef GenericOdometry<Values, PoseKey> Odometry;

	} // namespace simulated2DOriented
} // namespace gtsam
