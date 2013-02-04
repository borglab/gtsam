/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file ImuBias.h
 * @date  Feb 2, 2012
 * @author Vadim Indelman, Stephen Williams
 */

#pragma once

#include <boost/serialization/nvp.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/DerivedValue.h>
#include <gtsam/geometry/Pose3.h>

/*
 * NOTES:
 * - Earth-rate correction:
 * 		+ Currently the user should supply R_ECEF_to_G, which is the rotation from ECEF to Local-Level system (NED or ENU as defened by the user).
 * 		+ R_ECEF_to_G can be calculated by approximated values of latitude and longitude of the system.
 *		+ A relatively small distance is traveled w.r.t. to initial pose is assumed, since R_ECEF_to_G is constant.
 *		    Otherwise, R_ECEF_to_G should be updated each time using the current lat-lon.
 *
 *	- Currently, an empty constructed is not enabled so that the user is forced to specify R_ECEF_to_G.
 */

namespace gtsam {



	/// All noise models live in the noiseModel namespace
	namespace imuBias {

  class ConstantBias : public DerivedValue<ConstantBias> {
	private:
		Vector bias_acc_;
		Vector bias_gyro_;

	public:

    ConstantBias():
      bias_acc_(Vector_(3, 0.0, 0.0, 0.0)),  bias_gyro_(Vector_(3, 0.0, 0.0, 0.0)) {
    }

    ConstantBias(const Vector& bias_acc, const Vector& bias_gyro):
      bias_acc_(bias_acc),  bias_gyro_(bias_gyro) {
    }

		Vector CorrectAcc(Vector measurment, boost::optional<Matrix&> H=boost::none) const {
		  if (H){
				Matrix zeros3_3(zeros(3,3));
				Matrix m_eye3(-eye(3));

				*H = collect(2, &m_eye3, &zeros3_3);
			}

			return measurment - bias_acc_;
		}


		Vector CorrectGyro(Vector measurment, boost::optional<Matrix&> H=boost::none) const {
			if (H){
				Matrix zeros3_3(zeros(3,3));
				Matrix m_eye3(-eye(3));

				*H = collect(2, &zeros3_3, &m_eye3);
			}

			return measurment - bias_gyro_;
		}

		// H1: Jacobian w.r.t. IMUBias
		// H2: Jacobian w.r.t. pose
		Vector CorrectGyroWithEarthRotRate(Vector measurement, const Pose3& pose, const Vector& w_earth_rate_G,
				boost::optional<Matrix&> H1=boost::none, boost::optional<Matrix&> H2=boost::none) const {

			Matrix R_G_to_I( pose.rotation().matrix().transpose() );
			Vector w_earth_rate_I = R_G_to_I * w_earth_rate_G;

			if (H1){
				Matrix zeros3_3(zeros(3,3));
				Matrix m_eye3(-eye(3));

				*H1 = collect(2, &zeros3_3, &m_eye3);
			}

			if (H2){
				Matrix zeros3_3(zeros(3,3));
				Matrix H = -skewSymmetric(w_earth_rate_I);

				*H2 = collect(2, &H, &zeros3_3);
			}

			//TODO: Make sure H2 is correct.

			return measurement - bias_gyro_ - w_earth_rate_I;

//			Vector bias_gyro_temp(Vector_(3, -bias_gyro_(0), bias_gyro_(1), bias_gyro_(2)));
//			return measurement - bias_gyro_temp - R_G_to_I * w_earth_rate_G;
		}

		/** Expmap around identity */
		static inline ConstantBias Expmap(const Vector& v) { return ConstantBias(v.head(3), v.tail(3)); }

		/** Logmap around identity - just returns with default cast back */
		static inline Vector Logmap(const ConstantBias& p) { return concatVectors(2, &p.bias_acc_, &p.bias_gyro_); }

		/** Update the LieVector with a tangent space update */
		inline ConstantBias retract(const Vector& v) const { return ConstantBias(bias_acc_ + v.head(3), bias_gyro_ + v.tail(3)); }

		/** @return the local coordinates of another object */
		inline Vector localCoordinates(const ConstantBias& t2) const {
			Vector delta_acc(t2.bias_acc_ - bias_acc_);
			Vector delta_gyro(t2.bias_gyro_ - bias_gyro_);
			return concatVectors(2, &delta_acc, &delta_gyro);
		}

		/** Returns dimensionality of the tangent space */
		inline size_t dim() const { return this->bias_acc_.size() + this->bias_gyro_.size(); }

		/// print with optional string
		void print(const std::string& s = "") const {
			// explicit printing for now.
			std::cout << s + ".bias_acc [" << bias_acc_.transpose() << "]" << std::endl;
			std::cout << s + ".bias_gyro [" << bias_gyro_.transpose() << "]" << std::endl;
		}

		/** equality up to tolerance */
		inline bool equals(const ConstantBias& expected, double tol=1e-5) const {
			return gtsam::equal(bias_acc_, expected.bias_acc_, tol) && gtsam::equal(bias_gyro_, expected.bias_gyro_, tol);
		}

		/** get bias_acc */
		const Vector& bias_acc() const { return bias_acc_; }

		/** get bias_gyro */
		const Vector& bias_gyro() const { return bias_gyro_; }


		ConstantBias compose(const ConstantBias& b2,
				boost::optional<gtsam::Matrix&> H1=boost::none,
				boost::optional<gtsam::Matrix&> H2=boost::none) const {
			if(H1) *H1 = eye(dim());
			if(H2) *H2 = eye(b2.dim());

			return ConstantBias(bias_acc_ + b2.bias_acc_, bias_gyro_ + b2.bias_gyro_);
		}

		/** between operation */
		ConstantBias between(const ConstantBias& b2,
				boost::optional<gtsam::Matrix&> H1=boost::none,
				boost::optional<gtsam::Matrix&> H2=boost::none) const {
			if(H1) *H1 = -eye(dim());
			if(H2) *H2 = eye(b2.dim());
			return ConstantBias(b2.bias_acc_ - bias_acc_, b2.bias_gyro_ - bias_gyro_);
		}

		/** invert the object and yield a new one */
		inline ConstantBias inverse(boost::optional<gtsam::Matrix&> H=boost::none) const {
			if(H) *H = -eye(dim());

			return ConstantBias(-1.0 * bias_acc_, -1.0 * bias_gyro_);
		}



	}; // ConstantBias class


	} // namespace ImuBias

} // namespace gtsam


