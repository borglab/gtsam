/**
 *  @file  BetweenFactor.h
 *  @authors Frank Dellaert, Viorela Ila
 **/
#pragma once

#include <ostream>

#include "NonlinearFactor.h"
#include "GaussianFactor.h"
#include "Lie.h"
#include "Matrix.h"

namespace gtsam {

	/**
	 * A class for a measurement predicted by "between(config[key1],config[key2])"
	 * T is the Lie group type, Config where the T's are gotten from
	 */
	template<class T, class Config>
	class BetweenFactor: public NonlinearFactor<Config> {

	private:

		T measured_; /** The measurement */
		std::string key1_, key2_; /** The keys of the two poses, order matters */
		std::list<std::string> keys_; /** The keys as a list */
		Matrix square_root_inverse_covariance_; /** sqrt(inv(measurement_covariance)) */

	public:

		// shorthand for a smart pointer to a factor
		typedef typename boost::shared_ptr<BetweenFactor> shared_ptr;

		/** Constructor */
		BetweenFactor(const std::string& key1, const std::string& key2,
				const T& measured, const Matrix& measurement_covariance) :
			key1_(key1), key2_(key2), measured_(measured) {
			square_root_inverse_covariance_ = inverse_square_root(
					measurement_covariance);
			keys_.push_back(key1);
			keys_.push_back(key2);
		}

		/** implement functions needed for Testable */

		/** print */
		void print(const std::string& name) const {
			std::cout << name << std::endl;
			std::cout << "Factor " << std::endl;
			std::cout << "key1 " << key1_ << std::endl;
			std::cout << "key2 " << key2_ << std::endl;
			measured_.print("measured ");
			gtsam::print(square_root_inverse_covariance_, "MeasurementCovariance");
		}

		/** equals */
		bool equals(const NonlinearFactor<Config>& expected, double tol) const {
			return key1_ == expected.key1_ && key2_ == expected.key2_
					&& measured_.equals(expected.measured_, tol);
		}

		/** implement functions needed to derive from Factor */

		/** vector of errors */
		Vector error_vector(const Config& x) const {
			//z-h
			const T& p1 = x.get(key1_), p2 = x.get(key2_);
			T hx = between(p1,p2);
			// manifold equivalent of z-h(x) -> log(h(x),z)
			return square_root_inverse_covariance_ * logmap(hx,measured_);
		}

		/** keys as a list */
		inline std::list<std::string> keys() const { return keys_;}

		/** number of variables attached to this factor */
		inline std::size_t size() const { return 2;}

		/** linearize */
		boost::shared_ptr<GaussianFactor> linearize(const Config& x0) const {
			const T& p1 = x0.get(key1_), p2 = x0.get(key2_);
			Matrix A1 = Dbetween1(p1, p2);
			Matrix A2 = Dbetween2(p1, p2);
			Vector b = error_vector(x0); // already has sigmas in !
			boost::shared_ptr<GaussianFactor> linearized(new GaussianFactor(
					key1_, square_root_inverse_covariance_ * A1,
					key2_, square_root_inverse_covariance_ * A2, b, 1.0));
			return linearized;
		}
	};

} /// namespace gtsam
