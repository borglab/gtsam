/**
 * @file    simulated2DConstraints.h
 * @brief   measurement functions and constraint definitions for simulated 2D robot
 * @author  Alex Cunningham
 */

// \callgraph

#pragma once

#include <NonlinearConstraint.h>
#include <simulated2D.h>

// \namespace

namespace gtsam {

	namespace simulated2D {

		/**
		 * Unary constraint encoding a hard equality on a Point
		 */
		template<class Cfg = Config, class Key = PoseKey>
		struct GenericUnaryEqualityConstraint: public NonlinearConstraint1<Cfg, Key, Point2> {
			typedef NonlinearConstraint1<Cfg, Key, Point2> Base;
			typedef boost::shared_ptr<GenericUnaryEqualityConstraint<Cfg, Key> > shared_ptr;

			Point2 z_;

			GenericUnaryEqualityConstraint(const Point2& z, const Key& key, double mu = 1000.0) :
				Base(key, 2, mu), z_(z) {
			}

			Vector evaluateError(const Point2& x, boost::optional<Matrix&> H =
					boost::none) const {
				return (prior(x, H) - z_).vector();
			}

		};

		/**
		 * Binary constraint simulating "odometry" between two Poses
		 */
		template<class Cfg = Config, class Key = PoseKey>
		struct GenericOdoHardEqualityConstraint: public NonlinearConstraint2<Cfg, Key, Point2, Key,	Point2> {
			typedef NonlinearConstraint2<Cfg, Key, Point2, Key,	Point2> Base;
			typedef boost::shared_ptr<GenericOdoHardEqualityConstraint<Cfg,Key> > shared_ptr;
			Point2 z_;

			GenericOdoHardEqualityConstraint(
					const Point2& z, const Key& i1, const Key& i2, double mu = 1000.0) :
				Base (i1, i2, 2, mu), z_(z) {
			}

			Vector evaluateError(const Point2& x1, const Point2& x2, boost::optional<
					Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none) const {
				return (odo(x1, x2, H1, H2) - z_).vector();
			}

		};

		/** Typedefs for regular use */
		typedef GenericUnaryEqualityConstraint<Config, PoseKey> UnaryEqualityConstraint;
		typedef GenericOdoHardEqualityConstraint<Config, PoseKey> OdoEqualityConstraint;

		/** Equality between variables */
		typedef NonlinearEquality2<Config, PoseKey, Point2> PoseEqualityConstraint;
		typedef NonlinearEquality2<Config, PointKey, Point2> PointEqualityConstraint;

	} // namespace simulated2D
} // namespace gtsam
