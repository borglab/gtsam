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

		namespace equality_constraints {

			/** Typedefs for regular use */
			typedef NonlinearEquality1<Config, PoseKey, Point2> UnaryEqualityConstraint;
			typedef BetweenConstraint<Config, PoseKey, Point2> OdoEqualityConstraint;

			/** Equality between variables */
			typedef NonlinearEquality2<Config, PoseKey, Point2> PoseEqualityConstraint;
			typedef NonlinearEquality2<Config, PointKey, Point2> PointEqualityConstraint;

		} // \namespace equality_constraints

		namespace inequality_constraints {

			/**
			 * Unary inequality constraint forcing a coordinate to be greater than a fixed value (c)
			 */
			template<class Cfg, class Key, unsigned int Idx>
			struct ScalarInequalityConstraint1: public NonlinearConstraint1<Cfg, Key, Point2> {
				typedef NonlinearConstraint1<Cfg, Key, Point2> Base;
				typedef boost::shared_ptr<ScalarInequalityConstraint1<Cfg, Key, Idx> > shared_ptr;

				double c_; 	   /// min value of the selected coordinate

				ScalarInequalityConstraint1(const Key& key, double c, double mu = 1000.0) :
					Base(key, 1, mu), c_(c) {
				}

				/** active when constraint not met */
				virtual bool active(const Cfg& c) const {
					return c[this->key_].vector()(Idx) <= c_; // greater than or equals to avoid zigzagging
				}

				Vector evaluateError(const Point2& x, boost::optional<Matrix&> H =
						boost::none) const {
					if (H) {
						Matrix D = zeros(1, 2);
						D(0, Idx) = 1.0;
						*H = D;
					}
					return Vector_(1, x.vector()(Idx) - c_);
				}

			};

			/** typedefs for use with simulated2D systems */
			typedef ScalarInequalityConstraint1<Config, PoseKey, 0> PoseXInequality;
			typedef ScalarInequalityConstraint1<Config, PoseKey, 1> PoseYInequality;

		} // \namespace inequality_constraints

	} // \namespace simulated2D
} // \namespace gtsam
