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

// \namespace

namespace gtsam {
namespace simulated3D {

/**
 * This is a linear SLAM domain where both poses and landmarks are
 * 3D points, without rotation. The structure and use is based on
 * the simulated2D domain.
 */

typedef gtsam::TypedSymbol<Point3, 'x'> PoseKey;
typedef gtsam::TypedSymbol<Point3, 'l'> PointKey;

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

/**
 * A prior factor on a single linear robot pose
 */
struct PointPrior3D: public NonlinearFactor1<PoseKey> {

	Point3 z_; ///< The prior pose value for the variable attached to this factor

	/**
	 * Constructor for a prior factor
	 * @param z is the measured/prior position for the pose
	 * @param model is the measurement model for the factor (Dimension: 3)
	 * @param j is the key for the pose
	 */
	PointPrior3D(const Point3& z,
			const SharedNoiseModel& model, const PoseKey& j) :
				NonlinearFactor1<PoseKey> (model, j), z_(z) {
	}

	/**
	 * Evaluates the error at a given value of x,
	 * with optional derivatives.
	 * @param x is the current value of the variable
	 * @param H is an optional Jacobian matrix (Dimension: 3x3)
	 * @return Vector error between prior value and x (Dimension: 3)
	 */
	Vector evaluateError(const Point3& x, boost::optional<Matrix&> H =
			boost::none) {
		return (prior(x, H) - z_).vector();
	}
};

/**
 * Models a linear 3D measurement between 3D points
 */
struct Simulated3DMeasurement: public NonlinearFactor2<PoseKey, PointKey> {

	Point3 z_; ///< Linear displacement between a pose and landmark

	/**
	 * Creates a measurement factor with a given measurement
	 * @param z is the measurement, a linear displacement between poses and landmarks
	 * @param model is a measurement model for the factor (Dimension: 3)
	 * @param j1 is the pose key of the robot
	 * @param j2 is the point key for the landmark
	 */
	Simulated3DMeasurement(const Point3& z,
			const SharedNoiseModel& model, PoseKey& j1, PointKey j2) :
				NonlinearFactor2<PoseKey, PointKey> (
						model, j1, j2), z_(z) {
	}

	/**
	 * Error function with optional derivatives
	 * @param x1 a robot pose value
	 * @param x2 a landmark point value
	 * @param H1 is an optional Jacobian matrix in terms of x1 (Dimension: 3x3)
	 * @param H2 is an optional Jacobian matrix in terms of x2 (Dimension: 3x3)
	 * @return vector error between measurement and prediction (Dimension: 3)
	 */
	Vector evaluateError(const Point3& x1, const Point3& x2, boost::optional<
			Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none) {
		return (mea(x1, x2, H1, H2) - z_).vector();
	}
};

}} // namespace simulated3D
