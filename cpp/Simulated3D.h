/**
 * @file   Simulated3D.h
 * @brief  measurement functions and derivatives for simulated 3D robot
 * @author Alex Cunningham
 **/

// \callgraph

#pragma once

#include "Matrix.h"
#include "VectorConfig.h"
#include "NonlinearFactor.h"
#include "Key.h"

// \namespace

namespace simulated3D {

	typedef gtsam::VectorConfig VectorConfig;

	typedef gtsam::Symbol PoseKey;
	typedef gtsam::Symbol PointKey;

	/**
	 * Prior on a single pose
	 */
	Vector prior(const Vector& x);
	Matrix Dprior(const Vector& x);

	/**
	 * odometry between two poses
	 */
	Vector odo(const Vector& x1, const Vector& x2);
	Matrix Dodo1(const Vector& x1, const Vector& x2);
	Matrix Dodo2(const Vector& x1, const Vector& x2);

	/**
	 *  measurement between landmark and pose
	 */
	Vector mea(const Vector& x, const Vector& l);
	Matrix Dmea1(const Vector& x, const Vector& l);
	Matrix Dmea2(const Vector& x, const Vector& l);

	struct Point2Prior3D: public gtsam::NonlinearFactor1<VectorConfig, PoseKey,
			Vector> {

		Vector z_;

		Point2Prior3D(const Vector& z, double sigma, const PoseKey& j) :
			gtsam::NonlinearFactor1<VectorConfig, PoseKey, Vector>(sigma, j), z_(z) {
		}

		Vector evaluateError(const Vector& x, boost::optional<Matrix&> H =
				boost::none) {
			if (H) *H = Dprior(x);
			return prior(x) - z_;
		}
	};

	struct Simulated3DMeasurement: public gtsam::NonlinearFactor2<VectorConfig,
			PoseKey, Vector, PointKey, Vector> {

		Vector z_;

		Simulated3DMeasurement(const Vector& z, double sigma, PoseKey& j1,
				PointKey j2) :
			z_(z), gtsam::NonlinearFactor2<VectorConfig, PoseKey, Vector, PointKey,
					Vector>(sigma, j1, j2) {
		}

		Vector evaluateError(const Vector& x1, const Vector& x2, boost::optional<
				Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none) {
			if (H1) *H1 = Dmea1(x1, x2);
			if (H2) *H2 = Dmea2(x1, x2);
			return mea(x1, x2) - z_;
		}
	};

} // namespace simulated3D
