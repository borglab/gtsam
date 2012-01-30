/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    simulated2DConstraints.h
 * @brief   measurement functions and constraint definitions for simulated 2D robot
 * @author  Alex Cunningham
 */

// \callgraph

#pragma once

#include <gtsam/base/numericalDerivative.h>

#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/BoundingConstraint.h>
#include <gtsam/slam/simulated2D.h>

// \namespace

namespace gtsam {

	namespace simulated2D {

		namespace equality_constraints {

			/** Typedefs for regular use */
			typedef NonlinearEquality1<PoseKey> UnaryEqualityConstraint;
			typedef NonlinearEquality1<PointKey> UnaryEqualityPointConstraint;
			typedef BetweenConstraint<PoseKey> OdoEqualityConstraint;

			/** Equality between variables */
			typedef NonlinearEquality2<PoseKey> PoseEqualityConstraint;
			typedef NonlinearEquality2<PointKey> PointEqualityConstraint;

		} // \namespace equality_constraints

		namespace inequality_constraints {

			/**
			 * Unary inequality constraint forcing a coordinate to be greater/less than a fixed value (c)
			 * @tparam VALUES is the values structure for the graph
			 * @tparam KEY is the key type for the variable constrained
			 * @tparam IDX is an index in tangent space to constrain, must be less than KEY::VALUE::Dim()
			 */
			template<class KEY, unsigned int IDX>
			struct ScalarCoordConstraint1: public BoundingConstraint1<KEY> {
				typedef BoundingConstraint1<KEY> Base;  ///< Base class convenience typedef
				typedef boost::shared_ptr<ScalarCoordConstraint1<KEY, IDX> > shared_ptr; ///< boost::shared_ptr convenience typedef
				typedef typename KEY::Value Point; ///< Constrained variable type

				virtual ~ScalarCoordConstraint1() {}

				/**
				 * Constructor for constraint
				 * @param key is the label for the constrained variable
				 * @param c is the measured value for the fixed coordinate
				 * @param isGreaterThan is a flag to set inequality as greater than or less than
				 * @param mu is the penalty function gain
				 */
				ScalarCoordConstraint1(const KEY& key, double c,
						bool isGreaterThan, double mu = 1000.0) :
					Base(key, c, isGreaterThan, mu) {
				}

				/**
				 * Access function for the constrained index
				 * @return the index for the constrained coordinate
				 */
				inline unsigned int index() const { return IDX; }

				/**
				 * extracts a single value from the point to compute error
				 * @param x is the estimate of the constrained variable being evaluated
				 * @param H is an optional Jacobian, linearized at x
				 */
				virtual double value(const Point& x, boost::optional<Matrix&> H =
						boost::none) const {
					if (H) {
						Matrix D = zeros(1, x.dim());
						D(0, IDX) = 1.0;
						*H = D;
					}
					return Point::Logmap(x)(IDX);
				}
			};

			/** typedefs for use with simulated2D systems */
			typedef ScalarCoordConstraint1<PoseKey, 0> PoseXInequality; ///< Simulated2D domain example factor constraining X
			typedef ScalarCoordConstraint1<PoseKey, 1> PoseYInequality; ///< Simulated2D domain example factor constraining Y

			/**
			 * Trait for distance constraints to provide distance
			 * @tparam T1 is a Lie value for which distance functions exist
			 * @tparam T2 is a Lie value for which distance functions exist
			 * @param a is the first Lie element
			 * @param b is the second Lie element
			 * @return a scalar distance between values
			 */
			template<class T1, class T2>
			double range_trait(const T1& a, const T2& b) { return a.dist(b); }

			/**
			 * Binary inequality constraint forcing the range between points
			 * to be less than or equal to a bound
			 * @tparam VALUES is the variable set for the graph
			 * @tparam KEY is the type of the keys for the variables constrained
			 */
			template<class KEY>
			struct MaxDistanceConstraint : public BoundingConstraint2<KEY, KEY> {
				typedef BoundingConstraint2<KEY, KEY> Base;  ///< Base class for factor
				typedef typename KEY::Value Point; ///< Type of variable constrained

				virtual ~MaxDistanceConstraint() {}

				/**
				 * Primary constructor for factor
				 * @param key1 is the first variable key
				 * @param key2 is the second variable key
				 * @param range_bound is the maximum range allowed between the variables
				 * @param mu is the gain for the penalty function
				 */
				MaxDistanceConstraint(const KEY& key1, const KEY& key2, double range_bound, double mu = 1000.0)
					: Base(key1, key2, range_bound, false, mu) {}

				/**
				 * computes the range with derivatives
				 * @param x1 is the first variable value
				 * @param x2 is the second variable value
				 * @param H1 is an optional Jacobian in x1
				 * @param H2 is an optional Jacobian in x2
				 * @return the distance between the variables
				 */
				virtual double value(const Point& x1, const Point& x2,
						boost::optional<Matrix&> H1 = boost::none,
						boost::optional<Matrix&> H2 = boost::none) const {
					if (H1) *H1 = numericalDerivative21(range_trait<Point,Point>, x1, x2, 1e-5);
					if (H1) *H2 = numericalDerivative22(range_trait<Point,Point>, x1, x2, 1e-5);
					return range_trait(x1, x2);
				}
			};

			typedef MaxDistanceConstraint<PoseKey> PoseMaxDistConstraint; ///< Simulated2D domain example factor

			/**
			 * Binary inequality constraint forcing a minimum range
			 * NOTE: this is not a convex function!  Be careful with initialization.
			 * @tparam VALUES is the variable set for the graph
			 * @tparam XKEY is the type of the pose key constrained
			 * @tparam PKEY is the type of the point key constrained
			 */
			template<class XKEY, class PKEY>
			struct MinDistanceConstraint : public BoundingConstraint2<XKEY, PKEY> {
				typedef BoundingConstraint2<XKEY, PKEY> Base; ///< Base class for factor
				typedef typename XKEY::Value Pose; ///< Type of pose variable constrained
				typedef typename PKEY::Value Point; ///< Type of point variable constrained

				virtual ~MinDistanceConstraint() {}

				/**
				 * Primary constructor for factor
				 * @param key1 is the first variable key
				 * @param key2 is the second variable key
				 * @param range_bound is the minimum range allowed between the variables
				 * @param mu is the gain for the penalty function
				 */
				MinDistanceConstraint(const XKEY& key1, const PKEY& key2,
						double range_bound, double mu = 1000.0)
					: Base(key1, key2, range_bound, true, mu) {}

				/**
				 * computes the range with derivatives
				 * @param x1 is the first variable value
				 * @param x2 is the second variable value
				 * @param H1 is an optional Jacobian in x1
				 * @param H2 is an optional Jacobian in x2
				 * @return the distance between the variables
				 */
				virtual double value(const Pose& x1, const Point& x2,
						boost::optional<Matrix&> H1 = boost::none,
						boost::optional<Matrix&> H2 = boost::none) const {
					if (H1) *H1 = numericalDerivative21(range_trait<Pose,Point>, x1, x2, 1e-5);
					if (H1) *H2 = numericalDerivative22(range_trait<Pose,Point>, x1, x2, 1e-5);
					return range_trait(x1, x2);
				}
			};

			typedef MinDistanceConstraint<PoseKey, PointKey> LandmarkAvoid; ///< Simulated2D domain example factor


		} // \namespace inequality_constraints

	} // \namespace simulated2D
} // \namespace gtsam
