/**
 * @file    UrbanOdometry.h
 * @brief   A Nonlinear Factor, specialized for Urban application
 * @author  Frank Dellaert
 * @author  Viorela Ila
 */

#pragma once

#include "UrbanFactor.h"

namespace gtsam {

	class UrbanOdometry: public UrbanFactor {
	private:
		std::string key1_, key2_; /** The keys of the two poses, order matters */
		Matrix square_root_inverse_covariance_; /** sqrt(inv(measurement_covariance)) */

	public:

		typedef boost::shared_ptr<UrbanOdometry> shared_ptr; // shorthand for a smart pointer to a factor

		UrbanOdometry(const std::string& key1, const std::string& key2,
				const Vector& measured, const Matrix& measurement_covariance) :
					UrbanFactor(measured, 1.0),
			key1_(key1), key2_(key2) {
			square_root_inverse_covariance_ = inverse_square_root(
					measurement_covariance);
		}

		/** implement functions needed for Testable */
		void print(const std::string& name) const {
			std::cout << name << std::endl;
			std::cout << "Factor " << std::endl;
			std::cout << "key1 " << key1_ << std::endl;
			std::cout << "key2 " << key2_ << std::endl;
			gtsam::print(z_,"measured ");
			gtsam::print(square_root_inverse_covariance_, "square_root_inverse_covariance");
		}

		bool equals(const NonlinearFactor<UrbanConfig>& expected, double tol) const {
			return false;
		}

		/** implement functions needed to derive from Factor */
		Vector error_vector(const UrbanConfig& config) const {
			return zero(6);
		}

		std::list<std::string> keys() const {
			std::list<std::string> l;
			l.push_back(key1_);
			l.push_back(key2_);
			return l;
		}

		std::size_t size() const {
			return 2;
		}

		/** linearize */
		boost::shared_ptr<GaussianFactor> linearize(const UrbanConfig& config) const {
			Vector b = zero(6);
			Matrix H1 = zeros(6,6);
			Matrix H2 = zeros(6,6);
			boost::shared_ptr<GaussianFactor> linearized(new GaussianFactor(key1_,
					square_root_inverse_covariance_ * H1, key2_,
					square_root_inverse_covariance_ * H2, b, 1.0));
			return linearized;
		}
	};

} /// namespace gtsam

