/**
 * @file    simulated2D.h
 * @brief   measurement functions and derivatives for simulated 2D robot
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

#include "VectorConfig.h"
#include "NonlinearFactor.h"

// \namespace

namespace simulated2D {

	typedef gtsam::VectorConfig VectorConfig;
	typedef std::string PoseKey;
	typedef std::string PointKey;

	/**
	 * Prior on a single pose, and optional derivative version
	 */
	inline Vector prior(const Vector& x) {return x;}
	Vector prior(const Vector& x, boost::optional<Matrix&> H = boost::none);

	/**
	 * odometry between two poses, and optional derivative version
	 */
	inline Vector odo(const Vector& x1, const Vector& x2) {return x2-x1;}
	Vector odo(const Vector& x1, const Vector& x2, boost::optional<Matrix&> H1 =
			boost::none, boost::optional<Matrix&> H2 = boost::none);

	/**
	 *  measurement between landmark and pose, and optional derivative version
	 */
	inline Vector mea(const Vector& x, const Vector& l) {return l-x;}
	Vector mea(const Vector& x, const Vector& l, boost::optional<Matrix&> H1 =
			boost::none, boost::optional<Matrix&> H2 = boost::none);

	/**
	 * Unary factor encoding a soft prior on a vector
	 */
	struct Prior: public gtsam::NonlinearFactor1<VectorConfig, std::string,
			Vector> {

		Vector z_;

		Prior(const Vector& z, double sigma, const std::string& key) :
			gtsam::NonlinearFactor1<VectorConfig, std::string, Vector>(sigma, key),
					z_(z) {
		}

		Vector evaluateError(const Vector& x, boost::optional<Matrix&> H =
				boost::none) const {
			return prior(x, H) - z_;
		}

	};

	/**
	 * Binary factor simulating "odometry" between two Vectors
	 */
	struct Odometry: public gtsam::NonlinearFactor2<VectorConfig, PoseKey,
			Vector, PointKey, Vector> {
		Vector z_;

		Odometry(const Vector& z, double sigma, const std::string& j1,
				const std::string& j2) :
			z_(z), gtsam::NonlinearFactor2<VectorConfig, PoseKey, Vector, PointKey,
					Vector>(sigma, j1, j2) {
		}

		Vector evaluateError(const Vector& x1, const Vector& x2, boost::optional<
				Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none) const {
			return odo(x1, x2, H1, H2) - z_;
		}

	};

	/**
	 * Binary factor simulating "measurement" between two Vectors
	 */
	struct Measurement: public gtsam::NonlinearFactor2<VectorConfig, PoseKey,
			Vector, PointKey, Vector> {

		Vector z_;

		Measurement(const Vector& z, double sigma, const std::string& j1,
				const std::string& j2) :
			z_(z), gtsam::NonlinearFactor2<VectorConfig, PoseKey, Vector, PointKey,
					Vector>(sigma, j1, j2) {
		}

		Vector evaluateError(const Vector& x1, const Vector& x2, boost::optional<
				Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 = boost::none) const {
			return mea(x1, x2, H1, H2) - z_;
		}

	};

} // namespace simulated2D
