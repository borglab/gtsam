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

namespace gtsam {
namespace simulated3D {

	typedef VectorConfig VectorConfig;

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

	struct Point2Prior3D: public NonlinearFactor1<VectorConfig, PoseKey,
			Vector> {

		Vector z_;

		Point2Prior3D(const Vector& z,
					const SharedGaussian& model, const PoseKey& j) :
				NonlinearFactor1<VectorConfig, PoseKey, Vector> (model, j), z_(z) {
			}

		Vector evaluateError(const Vector& x, boost::optional<Matrix&> H =
				boost::none) {
			if (H) *H = Dprior(x);
			return prior(x) - z_;
		}
	};

	struct Simulated3DMeasurement: public NonlinearFactor2<VectorConfig,
			PoseKey, Vector, PointKey, Vector> {

		Vector z_;

		Simulated3DMeasurement(const Vector& z,
					const SharedGaussian& model, PoseKey& j1, PointKey j2) :
				z_(z),
						NonlinearFactor2<VectorConfig, PoseKey, Vector, PointKey, Vector> (
								model, j1, j2) {
			}

		Vector evaluateError(const Vector& x1, const Vector& x2, boost::optional<
				Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none) {
			if (H1) *H1 = Dmea1(x1, x2);
			if (H2) *H2 = Dmea2(x1, x2);
			return mea(x1, x2) - z_;
		}
	};

}} // namespace simulated3D
