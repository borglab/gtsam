/**
 * @file    simulated2D.h
 * @brief   measurement functions and derivatives for simulated 2D robot
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

#include "Point2.h"
#include "TupleConfig.h"
#include "NonlinearFactor.h"

// \namespace

namespace gtsam {

	namespace simulated2D {

		// Simulated2D robots have no orientation, just a position
		typedef TypedSymbol<Point2, 'x'> PoseKey;
		typedef TypedSymbol<Point2, 'l'> PointKey;
		typedef PairConfig<PoseKey, Point2, PointKey, Point2> Config;

		/**
		 * Prior on a single pose, and optional derivative version
		 */
		inline Point2 prior(const Point2& x) {
			return x;
		}
		Point2 prior(const Point2& x, boost::optional<Matrix&> H = boost::none);

		/**
		 * odometry between two poses, and optional derivative version
		 */
		inline Point2 odo(const Point2& x1, const Point2& x2) {
			return x2 - x1;
		}
		Point2 odo(const Point2& x1, const Point2& x2, boost::optional<Matrix&> H1 =
				boost::none, boost::optional<Matrix&> H2 = boost::none);

		/**
		 *  measurement between landmark and pose, and optional derivative version
		 */
		inline Point2 mea(const Point2& x, const Point2& l) {
			return l - x;
		}
		Point2 mea(const Point2& x, const Point2& l, boost::optional<Matrix&> H1 =
				boost::none, boost::optional<Matrix&> H2 = boost::none);

		/**
		 * Unary factor encoding a soft prior on a vector
		 */
		struct Prior: public NonlinearFactor1<Config, PoseKey, Point2> {

			Point2 z_;

			Prior(const Point2& z, const SharedGaussian& model, const PoseKey& key) :
				NonlinearFactor1<Config, PoseKey, Point2> (model, key), z_(z) {
			}

			Vector evaluateError(const Point2& x, boost::optional<Matrix&> H =
					boost::none) const {
				return (prior(x, H) - z_).vector();
			}

		};

		/**
		 * Binary factor simulating "odometry" between two Vectors
		 */
		struct Odometry: public NonlinearFactor2<Config, PoseKey, Point2, PoseKey,
				Point2> {
			Point2 z_;

			Odometry(const Point2& z, const SharedGaussian& model, const PoseKey& j1,
					const PoseKey& j2) :
				z_(z), NonlinearFactor2<Config, PoseKey, Point2, PoseKey, Point2> (
						model, j1, j2) {
			}

			Vector evaluateError(const Point2& x1, const Point2& x2, boost::optional<
					Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none) const {
				return (odo(x1, x2, H1, H2) - z_).vector();
			}

		};

		/**
		 * Binary factor simulating "measurement" between two Vectors
		 */
		struct Measurement: public NonlinearFactor2<Config, PoseKey, Point2,
				PointKey, Point2> {

			Point2 z_;

			Measurement(const Point2& z, const SharedGaussian& model,
					const PoseKey& j1, const PointKey& j2) :
				z_(z), NonlinearFactor2<Config, PoseKey, Point2, PointKey, Point2> (
						model, j1, j2) {
			}

			Vector evaluateError(const Point2& x1, const Point2& x2, boost::optional<
					Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none) const {
				return (mea(x1, x2, H1, H2) - z_).vector();
			}

		};

	} // namespace simulated2D
} // namespace gtsam
