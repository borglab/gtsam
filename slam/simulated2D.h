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
		typedef LieConfig<PoseKey, Point2> PoseConfig;
		typedef LieConfig<PointKey, Point2> PointConfig;
		typedef TupleConfig2<PoseConfig, PointConfig> Config;

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
		template<class Cfg = Config, class Key = PoseKey>
		struct GenericPrior: public NonlinearFactor1<Cfg, Key, Point2> {

			Point2 z_;

			GenericPrior(const Point2& z, const SharedGaussian& model, const Key& key) :
				NonlinearFactor1<Cfg, Key, Point2> (model, key), z_(z) {
			}

			Vector evaluateError(const Point2& x, boost::optional<Matrix&> H =
					boost::none) const {
				return (prior(x, H) - z_).vector();
			}

		};

		/**
		 * Binary factor simulating "odometry" between two Vectors
		 */
		template<class Cfg = Config, class Key = PoseKey>
		struct GenericOdometry: public NonlinearFactor2<Cfg, Key, Point2, Key,
				Point2> {
			Point2 z_;

			GenericOdometry(const Point2& z, const SharedGaussian& model,
					const Key& i1, const Key& i2) :
				NonlinearFactor2<Cfg, Key, Point2, Key, Point2> (model, i1, i2), z_(z) {
			}

			Vector evaluateError(const Point2& x1, const Point2& x2, boost::optional<
					Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none) const {
				return (odo(x1, x2, H1, H2) - z_).vector();
			}

		};

		/**
		 * Binary factor simulating "measurement" between two Vectors
		 */
		template<class Cfg = Config, class XKey = PoseKey, class LKey = PointKey>
		class GenericMeasurement: public NonlinearFactor2<Cfg, XKey, Point2, LKey,
				Point2> {
		public:

			Point2 z_;

			GenericMeasurement(const Point2& z, const SharedGaussian& model,
					const XKey& i, const LKey& j) :
				NonlinearFactor2<Cfg, XKey, Point2, LKey, Point2> (model, i, j), z_(z) {
			}

			Vector evaluateError(const Point2& x1, const Point2& x2, boost::optional<
					Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none) const {
				return (mea(x1, x2, H1, H2) - z_).vector();
			}

		};

		/** Typedefs for regular use */
		typedef GenericPrior<Config, PoseKey> Prior;
		typedef GenericOdometry<Config, PoseKey> Odometry;
		typedef GenericMeasurement<Config, PoseKey, PointKey> Measurement;

	} // namespace simulated2D
} // namespace gtsam
