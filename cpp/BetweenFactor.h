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
	template<class Config, class Key, class T>
	class BetweenFactor: public NonlinearFactor2<Config, Key, T, Key, T> {

	private:

		typedef NonlinearFactor2<Config, Key, T, Key, T> Base;

		T measured_; /** The measurement */
		Matrix square_root_inverse_covariance_; /** sqrt(inv(measurement_covariance)) */

	public:

		// shorthand for a smart pointer to a factor
		typedef typename boost::shared_ptr<BetweenFactor> shared_ptr;

		/** Constructor */
		BetweenFactor(const Key& key1, const Key& key2, const T& measured,
				const Matrix& measurement_covariance) :
			Base(1, key1, key2), measured_(measured) {
			square_root_inverse_covariance_ = inverse_square_root(
					measurement_covariance);
		}

		/** implement functions needed for Testable */

		/** print */
		void print(const std::string& s) const {
			Base::print(s);
			measured_.print("measured ");
			gtsam::print(square_root_inverse_covariance_, "MeasurementCovariance");
		}

		/** equals */
		bool equals(const NonlinearFactor<Config>& expected, double tol) const {
			const BetweenFactor<Config, Key, T> *e =
					dynamic_cast<const BetweenFactor<Config, Key, T>*> (&expected);
			return e != NULL && Base::equals(expected)
					&& this->measured_.equals(e->measured_, tol);
		}

		/** implement functions needed to derive from Factor */

		/** vector of errors */
		Vector evaluateError(const T& p1, const T& p2, boost::optional<Matrix&> H1 =
				boost::none, boost::optional<Matrix&> H2 = boost::none) const {
			// h - z
			T hx = between(p1, p2);
			// TODO should be done by noise model
			if (H1 || H2) {
				between(p1,p2,H1,H2);
				if (H1) *H1 = square_root_inverse_covariance_ * *H1;
				if (H2) *H2 = square_root_inverse_covariance_ * *H2;
			}
			// manifold equivalent of h(x)-z -> log(z,h(x))
			// TODO use noise model, error vector is not whitened yet
			return square_root_inverse_covariance_ * logmap(measured_, hx);
		}

		/** return the measured */
		inline const T measured() const {return measured_;}

		/** number of variables attached to this factor */
		inline std::size_t size() const { return 2;}
	};

} /// namespace gtsam
