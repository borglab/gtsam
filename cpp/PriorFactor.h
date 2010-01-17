/**
 *  @file  PriorFactor.h
 *  @authors Frank Dellaert
 **/
#pragma once

#include <ostream>
#include "NoiseModel.h"
#include "NonlinearFactor.h"
#include "Pose2.h"

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
		boost::shared_ptr<GaussianNoiseModel> noiseModel_;

	public:

		// shorthand for a smart pointer to a factor
		typedef typename boost::shared_ptr<PriorFactor> shared_ptr;

		/** Constructor */
		PriorFactor(const Key& key, const T& prior,
				const boost::shared_ptr<GaussianNoiseModel>& model) :
			Base(1.0, key), prior_(prior), noiseModel_(model) {
		}

		/** Constructor */
		PriorFactor(const Key& key, const T& prior, const Matrix& cov) :
			Base(1.0, key), prior_(prior), noiseModel_(GaussianNoiseModel::Covariance(cov)) {
		}

		/** implement functions needed for Testable */

		/** print */
		void print(const std::string& s) const {
			Base::print(s);
			prior_.print("prior");
			// Todo print NoiseModel
		}

		/** equals */
		bool equals(const NonlinearFactor<Config>& expected, double tol) const {
			const PriorFactor<Config, Key, T> *e = dynamic_cast<const PriorFactor<
					Config, Key, T>*> (&expected);
			return e != NULL && Base::equals(expected) && this->prior_.equals(
					e->prior_, tol);
			// Todo check NoiseModel
		}

		/** implement functions needed to derive from Factor */

		/** vector of errors */
		Vector evaluateError(const T& p, boost::optional<Matrix&> H = boost::none) const {
			if (H) (*H) = noiseModel_->R();
			// manifold equivalent of h(x)-z -> log(z,h(x))
			return noiseModel_->whiten(logmap(prior_, p));
		}
	};

} /// namespace gtsam
