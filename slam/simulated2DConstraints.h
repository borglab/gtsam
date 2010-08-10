/**
 * @file    simulated2DConstraints.h
 * @brief   measurement functions and constraint definitions for simulated 2D robot
 * @author  Alex Cunningham
 */

// \callgraph

#pragma once

#include <numericalDerivative.h> // TODO: remove

#include "NonlinearConstraint.h"
#include "BetweenConstraint.h"
#include "BoundingConstraint.h"
#include "simulated2D.h"

// \namespace

namespace gtsam {

	namespace simulated2D {

		namespace equality_constraints {

			/** Typedefs for regular use */
			typedef NonlinearEquality1<Config, PoseKey, Point2> UnaryEqualityConstraint;
			typedef NonlinearEquality1<Config, PointKey, Point2> UnaryEqualityPointConstraint;
			typedef BetweenConstraint<Config, PoseKey, Point2> OdoEqualityConstraint;

			/** Equality between variables */
			typedef NonlinearEquality2<Config, PoseKey, Point2> PoseEqualityConstraint;
			typedef NonlinearEquality2<Config, PointKey, Point2> PointEqualityConstraint;

		} // \namespace equality_constraints

		namespace inequality_constraints {

			/**
			 * Unary inequality constraint forcing a coordinate to be greater/less than a fixed value (c)
			 * Demo implementation: should be made more general using BoundingConstraint
			 */
			template<class Cfg, class Key, unsigned int Idx>
			struct ScalarCoordConstraint1: public BoundingConstraint1<Cfg, Key, Point2> {
				typedef BoundingConstraint1<Cfg, Key, Point2> Base;
				typedef boost::shared_ptr<ScalarCoordConstraint1<Cfg, Key, Idx> > shared_ptr;

				ScalarCoordConstraint1(const Key& key, double c,
						bool isGreaterThan, double mu = 1000.0) :
					Base(key, c, isGreaterThan, mu) {
				}

				inline unsigned int index() const { return Idx; }

				/** extracts a single value from the point */
				virtual double value(const Point2& x, boost::optional<Matrix&> H =
						boost::none) const {
					if (H) {
						Matrix D = zeros(1, 2);
						D(0, Idx) = 1.0;
						*H = D;
					}
					return x.vector()(Idx);
				}
			};

			/** typedefs for use with simulated2D systems */
			typedef ScalarCoordConstraint1<Config, PoseKey, 0> PoseXInequality;
			typedef ScalarCoordConstraint1<Config, PoseKey, 1> PoseYInequality;

			double range(const Point2& a, const Point2& b) { return a.dist(b); }

			/**
			 * Binary inequality constraint forcing the range between points
			 * to be less than or equal to a bound
			 */
			template<class Cfg, class Key>
			struct MaxDistanceConstraint : public BoundingConstraint2<Cfg, Key, Point2, Key, Point2> {
				typedef BoundingConstraint2<Cfg, Key, Point2, Key, Point2> Base;

				MaxDistanceConstraint(const Key& key1, const Key& key2, double range_bound, double mu = 1000.0)
					: Base(key1, key2, range_bound, false, mu) {}

				/** extracts a single scalar value with derivatives */
				virtual double value(const Point2& x1, const Point2& x2,
						boost::optional<Matrix&> H1 = boost::none,
						boost::optional<Matrix&> H2 = boost::none) const {
					if (H1) *H1 = numericalDerivative21(range, x1, x2, 1e-5);
					if (H1) *H2 = numericalDerivative22(range, x1, x2, 1e-5);
					return x1.dist(x2);
				}
			};

			typedef MaxDistanceConstraint<Config, PoseKey> PoseMaxDistConstraint;

			/**
			 * Binary inequality constraint forcing a minimum range
			 * NOTE: this is not a convex function!  Be careful with initialization.
			 */
			template<class Cfg, class XKey, class PKey>
			struct MinDistanceConstraint : public BoundingConstraint2<Cfg, XKey, Point2, PKey, Point2> {
				typedef BoundingConstraint2<Cfg, XKey, Point2, PKey, Point2> Base;

				MinDistanceConstraint(const XKey& key1, const PKey& key2, double range_bound, double mu = 1000.0)
					: Base(key1, key2, range_bound, true, mu) {}

				/** extracts a single scalar value with derivatives */
				virtual double value(const Point2& x1, const Point2& x2,
						boost::optional<Matrix&> H1 = boost::none,
						boost::optional<Matrix&> H2 = boost::none) const {
					if (H1) *H1 = numericalDerivative21(range, x1, x2, 1e-5);
					if (H1) *H2 = numericalDerivative22(range, x1, x2, 1e-5);
					return x1.dist(x2);
				}
			};

			typedef MinDistanceConstraint<Config, PoseKey, PointKey> LandmarkAvoid;


		} // \namespace inequality_constraints

	} // \namespace simulated2D
} // \namespace gtsam
