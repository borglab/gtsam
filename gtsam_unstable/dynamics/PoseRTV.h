/**
 * @file PoseRTV.h
 * @brief Pose3 with translational velocity
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/base/DerivedValue.h>
#include <gtsam/geometry/Pose3.h>

namespace gtsam {

/// Syntactic sugar to clarify components
typedef Point3 Velocity3;

/**
 * Robot state for use with IMU measurements
 * - contains translation, translational velocity and rotation
 */
class PoseRTV : public DerivedValue<PoseRTV> {
protected:

	Pose3 Rt_;
	Velocity3 v_;

public:
	// constructors - with partial versions
	PoseRTV() {}
	PoseRTV(const Point3& pt, const Rot3& rot, const Velocity3& vel)
	: Rt_(rot, pt), v_(vel) {}
	PoseRTV(const Rot3& rot, const Point3& pt, const Velocity3& vel)
	: Rt_(rot, pt), v_(vel) {}
	explicit PoseRTV(const Point3& pt)
	: Rt_(Rot3::identity(), pt) {}
	PoseRTV(const Pose3& pose, const Velocity3& vel)
	: Rt_(pose), v_(vel) {}
	explicit PoseRTV(const Pose3& pose)
	: Rt_(pose) {}

	/** build from components - useful for data files */
	PoseRTV(double roll, double pitch, double yaw, double x, double y, double z,
			double vx, double vy, double vz);

	/** build from single vector - useful for Matlab - in RtV format */
	explicit PoseRTV(const Vector& v);

	// access
	const Point3& t() const { return Rt_.translation(); }
	const Rot3& R() const { return Rt_.rotation(); }
	const Velocity3& v() const { return v_; }
	const Pose3& pose() const { return Rt_; }

	// longer function names
	const Point3& translation() const { return Rt_.translation(); }
	const Rot3& rotation() const { return Rt_.rotation(); }
	const Velocity3& velocity() const { return v_; }

	// Access to vector for ease of use with Matlab
	// and avoidance of Point3
	Vector vector() const;
	Vector translationVec() const { return Rt_.translation().vector(); }
	Vector velocityVec() const { return v_.vector(); }

	// testable
	bool equals(const PoseRTV& other, double tol=1e-6) const;
	void print(const std::string& s="") const;

	// Manifold
	static size_t Dim() { return 9; }
	size_t dim() const { return Dim(); }

	/**
	 * retract/unretract assume independence of components
	 * Tangent space parameterization:
	 * 	 - v(0-2): Rot3 (roll, pitch, yaw)
	 * 	 - v(3-5): Point3
	 * 	 - v(6-8): Translational velocity
	 */
	PoseRTV retract(const Vector& v) const;
	Vector localCoordinates(const PoseRTV& p) const;

	// Lie
	/**
	 * expmap/logmap are poor approximations that assume independence of components
	 * Currently implemented using the poor retract/unretract approximations
	 */
	static PoseRTV Expmap(const Vector& v);
	static Vector Logmap(const PoseRTV& p);

	PoseRTV inverse() const;

	PoseRTV compose(const PoseRTV& p) const;

	static PoseRTV identity() { return PoseRTV(); }

	/** Derivatives calculated numerically */
	PoseRTV between(const PoseRTV& p,
			boost::optional<Matrix&> H1=boost::none,
			boost::optional<Matrix&> H2=boost::none) const;

	// measurement functions
	/** Derivatives calculated numerically */
	double range(const PoseRTV& other,
			boost::optional<Matrix&> H1=boost::none,
			boost::optional<Matrix&> H2=boost::none) const;

	// IMU-specific

	/// Dynamics integrator for ground robots
	/// Always move from time 1 to time 2
	PoseRTV planarDynamics(double vel_rate, double heading_rate, double max_accel, double dt) const;

	/// Simulates flying robot with simple flight model
	/// Integrates state x1 -> x2 given controls
	/// x1 = {p1, r1, v1}, x2 = {p2, r2, v2}, all in global coordinates
	/// @return x2
	PoseRTV flyingDynamics(double pitch_rate, double heading_rate, double lift_control, double dt) const;

	/// General Dynamics update - supply control inputs in body frame
	PoseRTV generalDynamics(const Vector& accel, const Vector& gyro, double dt) const;

	/// Dynamics predictor for both ground and flying robots, given states at 1 and 2
	/// Always move from time 1 to time 2
	/// @return imu measurement, as [accel, gyro]
	Vector imuPrediction(const PoseRTV& x2, double dt) const;

	/// predict measurement and where Point3 for x2 should be, as a way
	/// of enforcing a velocity constraint
	/// This version splits out the rotation and velocity for x2
	Point3 translationIntegration(const Rot3& r2, const Velocity3& v2, double dt) const;

	/// predict measurement and where Point3 for x2 should be, as a way
	/// of enforcing a velocity constraint
	/// This version takes a full PoseRTV, but ignores the existing translation for x2
	inline Point3 translationIntegration(const PoseRTV& x2, double dt) const {
		return translationIntegration(x2.rotation(), x2.velocity(), dt);
	}

	/// @return a vector for Matlab compatibility
	inline Vector translationIntegrationVec(const PoseRTV& x2, double dt) const {
		return translationIntegration(x2, dt).vector();
	}

	/**
	 * Apply transform to this pose, with optional derivatives
	 * equivalent to:
	 * local = trans.transform_from(global, Dtrans, Dglobal)
	 *
	 * Note: the transform jacobian convention is flipped
	 */
	PoseRTV transformed_from(const Pose3& trans,
			boost::optional<Matrix&> Dglobal=boost::none,
			boost::optional<Matrix&> Dtrans=boost::none) const;

	// Utility functions

	/// RRTMbn - Function computes the rotation rate transformation matrix from
	/// body axis rates to euler angle (global) rates
	static Matrix RRTMbn(const Vector& euler);

	static Matrix RRTMbn(const Rot3& att);

	/// RRTMnb - Function computes the rotation rate transformation matrix from
	/// euler angle rates to body axis rates
	static Matrix RRTMnb(const Vector& euler);

	static Matrix RRTMnb(const Rot3& att);

private:
	/** Serialization function */
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version) {
		ar & BOOST_SERIALIZATION_NVP(Rt_);
		ar & BOOST_SERIALIZATION_NVP(v_);
	}
};

} // \namespace gtsam
