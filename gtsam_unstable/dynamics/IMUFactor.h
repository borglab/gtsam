/**
 * @file IMUFactor.h
 * @brief Factor to express an IMU measurement between dynamic poses
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/LieVector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam_unstable/dynamics/PoseRTV.h>

namespace gtsam {

/**
 * Class that represents integrating IMU measurements over time for dynamic systems
 * Templated to allow for different key types, but variables all
 * assumed to be PoseRTV
 */
template<class POSE>
class IMUFactor : public NoiseModelFactor2<POSE, POSE> {
public:
	typedef NoiseModelFactor2<POSE, POSE> Base;
	typedef IMUFactor<POSE> This;

protected:

	/** measurements from the IMU */
	Vector accel_, gyro_;
	double dt_; /// time between measurements

public:

	/** Standard constructor */
	IMUFactor(const Vector& accel, const Vector& gyro,
			double dt, const Key& key1, const Key& key2, const SharedNoiseModel& model)
	: Base(model, key1, key2), accel_(accel), gyro_(gyro), dt_(dt) {}

	/** Full IMU vector specification */
	IMUFactor(const Vector& imu_vector,
			double dt, const Key& key1, const Key& key2, const SharedNoiseModel& model)
	: Base(model, key1, key2), accel_(imu_vector.head(3)), gyro_(imu_vector.tail(3)), dt_(dt) {}

	virtual ~IMUFactor() {}

	/// @return a deep copy of this factor
	virtual gtsam::NonlinearFactor::shared_ptr clone() const {
		return boost::static_pointer_cast<gtsam::NonlinearFactor>(
				gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

	/** Check if two factors are equal */
	virtual bool equals(const NonlinearFactor& e, double tol = 1e-9) const {
		const This* const f = dynamic_cast<const This*>(&e);
		return f && Base::equals(e) &&
				equal_with_abs_tol(accel_, f->accel_, tol) &&
				equal_with_abs_tol(gyro_, f->gyro_, tol) &&
				fabs(dt_ - f->dt_) < tol;
	}

	void print(const std::string& s="", const gtsam::KeyFormatter& formatter = gtsam::DefaultKeyFormatter) const {
		std::string a = "IMUFactor: " + s;
		Base::print(a, formatter);
		gtsam::print(accel_, "accel");
		gtsam::print(gyro_, "gyro");
		std::cout << "dt: " << dt_ << std::endl;
	}

	// access
	const Vector& gyro() const { return gyro_; }
	const Vector& accel() const { return accel_; }
	Vector z() const { return concatVectors(2, &accel_, &gyro_); }

	/**
	 * Error evaluation with optional derivatives - calculates
	 *  z - h(x1,x2)
	 */
	virtual Vector evaluateError(const PoseRTV& x1, const PoseRTV& x2,
			boost::optional<Matrix&> H1 = boost::none,
			boost::optional<Matrix&> H2 = boost::none) const {
		const Vector meas = z();
		if (H1) *H1 = numericalDerivative21<LieVector, PoseRTV, PoseRTV>(
				boost::bind(This::predict_proxy, _1, _2, dt_, meas), x1, x2, 1e-5);
		if (H2) *H2 = numericalDerivative22<LieVector, PoseRTV, PoseRTV>(
				boost::bind(This::predict_proxy, _1, _2, dt_, meas), x1, x2, 1e-5);
		return predict_proxy(x1, x2, dt_, meas);
	}

	/** dummy version that fails for non-dynamic poses */
	virtual Vector evaluateError(const Pose3& x1, const Pose3& x2,
			boost::optional<Matrix&> H1 = boost::none,
			boost::optional<Matrix&> H2 = boost::none) const {
		assert(false); // no corresponding factor here
		return zero(x1.dim());
	}

private:
	/** copy of the measurement function formulated for numerical derivatives */
	static LieVector predict_proxy(const PoseRTV& x1, const PoseRTV& x2,
			double dt, const Vector& meas) {
		Vector hx = x1.imuPrediction(x2, dt);
		return LieVector(meas - hx);
	}
};

} // \namespace gtsam
