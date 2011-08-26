/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   Simulated3D.h
 * @brief  measurement functions and derivatives for simulated 3D robot
 * @author Alex Cunningham
 **/

// \callgraph

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Key.h>
#include <gtsam/nonlinear/TupleValues.h>

// \namespace

namespace gtsam {
namespace simulated3D {

	typedef gtsam::TypedSymbol<Point3, 'x'> PoseKey;
	typedef gtsam::TypedSymbol<Point3, 'l'> PointKey;

	typedef LieValues<PoseKey> PoseValues;
	typedef LieValues<PointKey> PointValues;
	typedef TupleValues2<PoseValues, PointValues> Values;

	/**
	 * Prior on a single pose
	 */
	Point3 prior(const Point3& x, boost::optional<Matrix&> H = boost::none);

	/**
	 * odometry between two poses
	 */
	Point3 odo(const Point3& x1, const Point3& x2,
			boost::optional<Matrix&> H1 = boost::none,
			boost::optional<Matrix&> H2 = boost::none);

	/**
	 *  measurement between landmark and pose
	 */
	Point3 mea(const Point3& x, const Point3& l,
			boost::optional<Matrix&> H1 = boost::none,
			boost::optional<Matrix&> H2 = boost::none);

	struct PointPrior3D: public NonlinearFactor1<Values, PoseKey> {

		Point3 z_;

		PointPrior3D(const Point3& z,
					const SharedNoiseModel& model, const PoseKey& j) :
				NonlinearFactor1<Values, PoseKey> (model, j), z_(z) {
			}

		Vector evaluateError(const Point3& x, boost::optional<Matrix&> H =
				boost::none) {
			return (prior(x, H) - z_).vector();
		}
	};

	struct Simulated3DMeasurement: public NonlinearFactor2<Values,
			PoseKey, PointKey> {

		Point3 z_;

		Simulated3DMeasurement(const Point3& z,
					const SharedNoiseModel& model, PoseKey& j1, PointKey j2) :
				NonlinearFactor2<Values, PoseKey, PointKey> (
								model, j1, j2), z_(z) {
			}

		Vector evaluateError(const Point3& x1, const Point3& x2, boost::optional<
				Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none) {
			return (mea(x1, x2, H1, H2) - z_).vector();
		}
	};

}} // namespace simulated3D
