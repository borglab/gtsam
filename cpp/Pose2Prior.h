/**
 *  @file  Pose2Prior.h
 *  @authors Michael Kaess
 **/
#pragma once

#include <map>
#include "NonlinearFactor.h"
#include "GaussianFactor.h"
#include "Pose2.h"
#include "Matrix.h"
#include "Key.h"
#include "Pose2Graph.h"
#include "ostream"

namespace gtsam {

	/**
	 * A class for a soft prior on any Lie type
	 * T is the Lie group type, Config where the T's are gotten from
	 */
	template<class Config, class Key, class T>
	class PriorFactor: public NonlinearFactor1<Config, Key, T> {

	private:

		typedef NonlinearFactor1<Config, Key, T> Base;

		T prior_; /** The measurement */
		Matrix square_root_inverse_covariance_; /** sqrt(inv(covariance)) */

	public:

		// shorthand for a smart pointer to a factor
		typedef typename boost::shared_ptr<PriorFactor> shared_ptr;

		/** Constructor */
		PriorFactor(const Key& key, const T& prior, const Matrix& covariance) :
			Base(1.0, key), prior_(prior) {
			square_root_inverse_covariance_ = inverse_square_root(covariance);
		}

		/** implement functions needed for Testable */

		/** print */
		void print(const std::string& s) const {
			Base::print(s);
			prior_.print("prior");
			gtsam::print(square_root_inverse_covariance_,
					"Square Root Inverse Covariance");
		}

		/** equals */
		bool equals(const NonlinearFactor<Config>& expected, double tol) const {
			const PriorFactor<Config, Key, T> *e = dynamic_cast<const PriorFactor<
					Config, Key, T>*> (&expected);
			return e != NULL && Base::equals(expected) && this->prior_.equals(
					e->prior_, tol);
		}

		/** implement functions needed to derive from Factor */

		/** vector of errors */
		Vector evaluateError(const T& p, boost::optional<Matrix&> H = boost::none) const {
			if (H) (*H) = square_root_inverse_covariance_;
			// manifold equivalent of h(x)-z -> log(z,h(x))
			return square_root_inverse_covariance_ * logmap(prior_, p);
		}
	};

	/** This is just a typedef now */
	typedef PriorFactor<Pose2Config, Pose2Config::Key, Pose2> Pose2Prior;

} /// namespace gtsam
