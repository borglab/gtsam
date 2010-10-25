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

#include <gtsam/geometry/Point2.h>
#include <gtsam/nonlinear/TupleValues.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

// \namespace

namespace gtsam {

	namespace simulated2D {

		// Simulated2D robots have no orientation, just a position
		typedef TypedSymbol<Point2, 'x'> PoseKey;
		typedef TypedSymbol<Point2, 'l'> PointKey;
		typedef LieValues<PoseKey> PoseValues;
		typedef LieValues<PointKey> PointValues;
		typedef TupleValues2<PoseValues, PointValues> Values;

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
		template<class CFG = Values, class KEY = PoseKey>
		struct GenericPrior: public NonlinearFactor1<CFG, KEY> {
			typedef boost::shared_ptr<GenericPrior<CFG, KEY> > shared_ptr;
			typedef typename KEY::Value Pose;
			Pose z_;

			GenericPrior(const Pose& z, const SharedGaussian& model, const KEY& key) :
				NonlinearFactor1<CFG, KEY> (model, key), z_(z) {
			}

			Vector evaluateError(const Pose& x, boost::optional<Matrix&> H =
					boost::none) const {
				return (prior(x, H) - z_).vector();
			}

		};

		/**
		 * Binary factor simulating "odometry" between two Vectors
		 */
		template<class CFG = Values, class KEY = PoseKey>
		struct GenericOdometry: public NonlinearFactor2<CFG, KEY, KEY> {
			typedef boost::shared_ptr<GenericOdometry<CFG, KEY> > shared_ptr;
			typedef typename KEY::Value Pose;
			Pose z_;

			GenericOdometry(const Pose& z, const SharedGaussian& model,
					const KEY& i1, const KEY& i2) :
				NonlinearFactor2<CFG, KEY, KEY> (model, i1, i2), z_(z) {
			}

			Vector evaluateError(const Pose& x1, const Pose& x2, boost::optional<
					Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none) const {
				return (odo(x1, x2, H1, H2) - z_).vector();
			}

		};

		/**
		 * Binary factor simulating "measurement" between two Vectors
		 */
		template<class CFG = Values, class XKEY = PoseKey, class LKEY = PointKey>
		class GenericMeasurement: public NonlinearFactor2<CFG, XKEY, LKEY> {
		public:
			typedef boost::shared_ptr<GenericMeasurement<CFG, XKEY, LKEY> > shared_ptr;
			typedef typename XKEY::Value Pose;
			typedef typename LKEY::Value Point;

			Point z_;

			GenericMeasurement(const Point& z, const SharedGaussian& model,
					const XKEY& i, const LKEY& j) :
				NonlinearFactor2<CFG, XKEY, LKEY> (model, i, j), z_(z) {
			}

			Vector evaluateError(const Pose& x1, const Point& x2, boost::optional<
					Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none) const {
				return (mea(x1, x2, H1, H2) - z_).vector();
			}

		};

		/** Typedefs for regular use */
		typedef GenericPrior<Values, PoseKey> Prior;
		typedef GenericOdometry<Values, PoseKey> Odometry;
		typedef GenericMeasurement<Values, PoseKey, PointKey> Measurement;

	} // namespace simulated2D
} // namespace gtsam
