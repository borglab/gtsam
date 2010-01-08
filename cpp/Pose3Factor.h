/**
 * @file    Pose3Factor.h
 * @brief   A Nonlinear Factor, specialized for Urban application
 * @author  Frank Dellaert
 * @author  Viorela Ila
 */

#pragma once

#include <map>
#include "NonlinearFactor.h"
#include "GaussianFactor.h"
#include "VectorConfig.h"
#include "Pose3.h"
#include "Matrix.h"

namespace gtsam {

	typedef VectorConfig Config;

	class Pose3Factor: public NonlinearFactor<Config> {
	private:
		std::string key1_, key2_; /** The keys of the two poses, order matters */
		Pose3 measured_;
		Matrix square_root_inverse_covariance_; /** sqrt(inv(measurement_covariance)) */

	public:

		typedef boost::shared_ptr<Pose3Factor> shared_ptr; // shorthand for a smart pointer to a factor

		Pose3Factor(const std::string& key1, const std::string& key2,
				const Pose3& measured, const Matrix& measurement_covariance) :
			key1_(key1), key2_(key2), measured_(measured) {
			square_root_inverse_covariance_ = inverse_square_root(
					measurement_covariance);
		}

		/** implement functions needed for Testable */
		void print(const std::string& name) const {
			std::cout << name << std::endl;
			std::cout << "Factor " << std::endl;
			std::cout << "key1 " << key1_ << std::endl;
			std::cout << "key2 " << key2_ << std::endl;
			measured_.print("measured ");
			gtsam::print(square_root_inverse_covariance_, "MeasurementCovariance");
		}
		bool equals(const NonlinearFactor<Config>& expected, double tol) const {
			return false;
		}

		/** implement functions needed to derive from Factor */
		Vector error_vector(const Config& config) const {
			//z-h
			Pose3 p1 = config.get(key1_), p2 = config.get(key2_);
			// todo: removed vector() from Pose3, so emulating it here!  This is incorrect!
			Pose3 betw = between(p1,p2);
			Vector r_diff = logmap(measured_.rotation()) - logmap(betw.rotation());
			Vector t_diff = logmap(measured_.translation()) - logmap(betw.translation());
			return concatVectors(2, &r_diff, &t_diff);
		}

		std::list<std::string> keys() const {
			std::list<std::string> l;
			return l;
		}
		std::size_t size() const {
			return 2;
		}

		/** linearize */
		boost::shared_ptr<GaussianFactor> linearize(const Config& config) const {
			Pose3 p1 = config.get(key1_), p2 = config.get(key2_);
            Pose3 betw = between(p1,p2);
            // todo: removed vector() from Pose3, so emulating it here!  This is incorrect!
            Vector r_diff = logmap(measured_.rotation()) - logmap(betw.rotation());
            Vector t_diff = logmap(measured_.translation()) - logmap(betw.translation());
			Vector b = concatVectors(2, &r_diff, &t_diff);
			Matrix H1 = Dbetween1(p1, p2);
			Matrix H2 = Dbetween2(p1, p2);
			boost::shared_ptr<GaussianFactor> linearized(new GaussianFactor(key1_,
					square_root_inverse_covariance_ * H1, key2_,
					square_root_inverse_covariance_ * H2, b, 1.0));
			return linearized;
		}
	};

} /// namespace gtsam

